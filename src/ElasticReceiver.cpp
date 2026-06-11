#include "ElasticReceiver.h"
#include <iostream>

ElasticReceiver::ElasticReceiver(size_t max_blocks)
    : _max_queue_size(max_blocks), _s(INVALID_SOCKET), _is_running(false), _aligned(false)
{
    _staging_buffer.reserve(_samples_per_ms / 2);
    _ring_capacity =
        _samples_per_ms * RING_DEPTH_MS;

    _sample_ring.resize(
        _ring_capacity);
}

ElasticReceiver::~ElasticReceiver()
{
    _is_running = false;
    if (_s != INVALID_SOCKET)
        closesocket(_s);
    WSACleanup();
}

bool ElasticReceiver::connect_to_relay(const char *ip, int port)
{
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        return false;

    _s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_s == INVALID_SOCKET)
        return false;

    // We do NOT bind. We are a client.
    // We just need to know where the Relay is.
    _relay_addr.sin_family = AF_INET;
    _relay_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &_relay_addr.sin_addr);

    // 1. Send the JOIN command
    uint32_t JOIN_CMD = 0x4A4F494E; // "JOIN"
    sendto(_s, (const char *)&JOIN_CMD, 4, 0, (struct sockaddr *)&_relay_addr, sizeof(_relay_addr));

    // 2. Small sleep to let the Relay process the subscription
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 3. Flush only what's currently in the OS buffer
    u_long mode = 1;
    ioctlsocket(_s, FIONBIO, &mode);
    std::vector<char> flush_buffer(4096);
    int flushed = 0;
    while (recv(_s, flush_buffer.data(), (int)flush_buffer.size(), 0) > 0)
    {
        flushed++;
    }
    mode = 0;
    ioctlsocket(_s, FIONBIO, &mode);

    std::cout << "[*] Handshake sent. Flushed " << flushed << " stale packets." << std::endl;

    _is_running = true;
    std::thread t(&ElasticReceiver::ingest_thread, this);
    t.detach();
    return true;
}

void ElasticReceiver::ingest_thread()
{
    std::vector<uint8_t> packet_raw(2048);
    const int H_SIZE = sizeof(RFE_Header_t);
    uint32_t current_fs_rate = 0;
    uint16_t total_packets_in_second = 0;
    uint16_t current_packet_in_second = 0;

    while (_is_running)
    {
        sockaddr_in src{};
        int addr_len = sizeof(src);
        int len = recvfrom(_s, (char *)packet_raw.data(), (int)packet_raw.size(), 0, (struct sockaddr *)&src, &addr_len);

        if (len < H_SIZE)
            continue;

        RFE_Header_t *hdr = (RFE_Header_t *)packet_raw.data();
        uint8_t *payload = packet_raw.data() + H_SIZE;
        size_t payload_len = len - H_SIZE;
        if (_last_seq_num == 0)
            _last_seq_num = hdr->seq_num;

        // Dynamic Sample Rate Detection
        if (hdr->fs_rate != current_fs_rate)
        {
            current_fs_rate = hdr->fs_rate;
            _samples_per_ms = (uint16_t)(current_fs_rate / 1000);
            _packets_per_ms = (uint8_t)(_samples_per_ms / 2046);
            _ring_capacity = _samples_per_ms * RING_DEPTH_MS;
            _sample_ring.clear();
            _sample_ring.resize(_ring_capacity);
            fprintf(stdout, "[*] Rate Encoded: %8u Hz (%hhu PktsPerMS)\n",
                    current_fs_rate, _packets_per_ms);
        }

        total_packets_in_second = (uint16_t)(hdr->fs_rate / 2046); // IF alway 1023 pkt
        current_packet_in_second = (hdr->sample_tick % hdr->fs_rate) / 2046;
        _ms_count = (uint16_t)(current_packet_in_second / _packets_per_ms);
        _ms_frac = (uint8_t)(current_packet_in_second % _packets_per_ms);

        // --- Alignment Logic (Millisecond Roll) ---
        if (!_aligned && _ms_frac == 0)
        {
            _aligned = true;
            fprintf(stdout, "[*] Fractional Millisecond Alignment: Tick %8u count %3d frac %1d\n",
                    hdr->sample_tick, _ms_count, _ms_frac);
        }

        // --- Data Staging Logic ---
        // This must be INSIDE the while loop to process every packet
        if (_aligned)
        {
            if (_staging_count == 0)
            {
                _staging_header = *hdr;
            }

            _staging_buffer.insert(_staging_buffer.end(), payload, payload + payload_len);
            _staging_count++;

            // Push to queue once a full millisecond is staged
            if (_staging_count >= _packets_per_ms)
            {
                std::lock_guard<std::mutex> lock(_mtx);
                _ready_queue.push_back({_staging_header, _staging_buffer});
                _last_unix_time = hdr->unix_time;
                _last_ms_count = _ms_count;
                _last_ms_frac = _ms_frac;
                unpack_to_ring(
                    _staging_buffer.data(),
                    _staging_buffer.size(),
                    _staging_header.sample_tick,
                    hdr->unix_time);
                /*fprintf(stdout,
                    "[DBG] staged=%zu expected=%u\n",
                    _staging_buffer.size(),
                    _samples_per_ms / 2); */
                if (_ready_queue.size() > _max_queue_size)
                {
                    _ready_queue.pop_front();
                }

                _staging_buffer.clear();
                _staging_count = 0;
                // printf("Ring write: %llu\n", _write_index);
            }
        }
    } // End of while (_is_running)
}

void ElasticReceiver::jump_to_latest_epoch()
{
    std::lock_guard<std::mutex> lock(_mtx);

    int latest_epoch_index = -1;
    // Walk backward from newest data to oldest
    for (int i = (int)_ready_queue.size() - 1; i >= 0; --i)
    {
        // Find the most recent "Top of the Millisecond" (frac 0)
        if ((_ready_queue[i].header.sample_tick % _samples_per_ms) == 0)
        {
            latest_epoch_index = i;
            break;
        }
    }

    if (latest_epoch_index > 0)
    {
        // Keep only the latest epoch and everything that came after it
        for (int i = 0; i < latest_epoch_index; ++i)
        {
            _ready_queue.pop_front();
        }
    }
}

bool ElasticReceiver::get_ms_blocks(uint8_t *out, RFE_Header_t &first_header, size_t num_ms)
{
    while (_is_running)
    {
        {
            std::lock_guard<std::mutex> lock(_mtx);
            if (_ready_queue.size() >= num_ms)
                break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!_is_running)
        return false;

    std::lock_guard<std::mutex> lock(_mtx);
    for (size_t i = 0; i < num_ms; ++i)
    {
        MillisecondBlock block = _ready_queue.front();
        if (i == 0)
            first_header = block.header;

        memcpy(out + (i * 8184), block.data.data(), 8184);
        _ready_queue.pop_front();
    }

    return true;
}

TimeTrio ElasticReceiver::get_time_trio()
{
    TimeTrio tme3;
    tme3.unixSecond = _last_unix_time;
    tme3.msCount = _last_ms_count;
    tme3.fracMS = _last_ms_frac;
    return tme3;
}

void ElasticReceiver::unpack_to_ring(
    const uint8_t *packed,
    size_t packed_count,
    uint32_t sample_tick,
    uint32_t unix_time)
{
    const UnpackEntry *lut =
        GetLUT_FNHN();

    std::lock_guard<std::mutex>
        lock(_ring_mtx);

    uint32_t tick =
        sample_tick;

    for (size_t i = 0;
         i < packed_count;
         ++i)
    {
        const auto &e =
            lut[packed[i]];

        size_t idx =
            _write_index %
            _ring_capacity;

        _sample_ring[idx].i =
            e.s0.i;
        _sample_ring[idx].q =
            e.s0.q;
        _sample_ring[idx]
            .sample_tick =
            tick++;
        _sample_ring[idx]
            .unix_time =
            unix_time;
        _sample_ring[idx]
            .sample_index =
            _global_sample_index++;
        _write_index++;

        idx =
            _write_index %
            _ring_capacity;

        _sample_ring[idx].i =
            e.s1.i;
        _sample_ring[idx].q =
            e.s1.q;
        _sample_ring[idx]
            .sample_tick =
            tick++;
        _sample_ring[idx]
            .unix_time =
            unix_time;
        _sample_ring[idx]
            .sample_index =
            _global_sample_index++;
        _write_index++;
    }
}

RingTimingStatus ElasticReceiver::get_timing_status(
    uint64_t point,
    size_t samples_per_ms) const
{
    std::lock_guard<std::mutex> lock(_ring_mtx);

    RingTimingStatus s = {};

    s.write_index = _write_index;
    s.capacity_samples = _ring_capacity;
    s.point = point;

    s.oldest_available =
        (_write_index > _ring_capacity)
            ? (_write_index - _ring_capacity)
            : 0;

    s.lag_ms =
        (double)((int64_t)_write_index - (int64_t)point) /
        (double)samples_per_ms;

    s.margin_ms =
        (double)((int64_t)point - (int64_t)s.oldest_available) /
        (double)samples_per_ms;

    s.ring_ms =
        (double)_ring_capacity /
        (double)samples_per_ms;

    return s;
}

bool ElasticReceiver::validate_ring_continuity(size_t lookback)
{
    std::lock_guard<std::mutex> lock(_ring_mtx);

    if (_write_index < 2 || _ring_capacity == 0)
        return true;

    uint64_t count =
        (_write_index < lookback)
            ? _write_index
            : (uint64_t)lookback;

    if (count < 2)
        return true;

    uint64_t start = _write_index - count;

    for (uint64_t i = start + 1; i < _write_index; ++i)
    {
        const RawSample &prev =
            _sample_ring[(i - 1) % _ring_capacity];

        const RawSample &curr =
            _sample_ring[i % _ring_capacity];

        uint32_t expected =
            (uint32_t)(prev.sample_index + 1u);

        uint32_t got =
            curr.sample_index;

        if (got != expected)
        {
            fprintf(stdout,
                    "\n[RING BREAK] "
                    "logical=%llu "
                    "prev=%I64u "
                    "expected=%u "
                    "got=%u\n",
                    i,
                    prev.sample_index,
                    expected,
                    got);

            return false;
        }
    }

    return true;
}

bool ElasticReceiver::get_window(
    uint64_t start_index,
    RawSample *&out_ptr,
    unsigned int count,
    std::vector<RawSample> &scratch)
{
    std::lock_guard<std::mutex> lock(_ring_mtx);

    if (count == 0 || _ring_capacity == 0)
        return false;

    if (start_index + count > _write_index)
        return false;

    if (_write_index > _ring_capacity &&
        start_index < (_write_index - _ring_capacity))
        return false;

    scratch.resize(count);

    for (unsigned int i = 0; i < count; ++i)
    {
        scratch[i] =
            _sample_ring[(start_index + i) % _ring_capacity];
    }

    out_ptr = scratch.data();
    return true;
}