#include "ElasticReceiver.h"
#include <iostream>

ElasticReceiver::ElasticReceiver(size_t max_blocks) 
    : _max_queue_size(max_blocks), _s(INVALID_SOCKET), _is_running(false), _aligned(false) {
    _staging_buffer.reserve(8184);
}

ElasticReceiver::~ElasticReceiver() {
    _is_running = false;
    if (_s != INVALID_SOCKET) closesocket(_s);
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

void ElasticReceiver::ingest_thread() {
    std::vector<uint8_t> packet_raw(2048);
    const int H_SIZE = sizeof(RFE_Header_t);
    uint32_t current_fs_rate = 0;

    while (_is_running) {
        sockaddr_in src{};
        int addr_len = sizeof(src);
        int len = recvfrom(_s, (char*)packet_raw.data(), (int)packet_raw.size(), 0, (struct sockaddr*)&src, &addr_len);
        
        if (len < H_SIZE) continue;

        RFE_Header_t* hdr = (RFE_Header_t*)packet_raw.data();
        uint8_t* payload = packet_raw.data() + H_SIZE;
        size_t payload_len = len - H_SIZE;

        // Dynamic Sample Rate Detection
        if (hdr->fs_rate != current_fs_rate) {
            current_fs_rate = hdr->fs_rate;
            _samples_per_ms = current_fs_rate / 1000;
            _packets_per_ms = (uint8_t)(_samples_per_ms / 2046);
            std::cout << "[*] Rate Detected: " << current_fs_rate 
                      << " Hz (" << (int)_packets_per_ms << " ppm)" << std::endl;
        }

        // --- Alignment Logic (Millisecond Roll) ---
        uint32_t packet_index = (hdr->sample_tick % _samples_per_ms) / 2046;
        if (!_aligned && packet_index == 0) {
            _aligned = true;
            std::cout << "[+] EPOCH ALIGNED: Tick " << hdr->sample_tick << std::endl;
        }

        // --- Data Staging Logic ---
        // This must be INSIDE the while loop to process every packet
        if (_aligned) {
            if (_staging_count == 0) {
                _staging_header = *hdr;
            }
            
            _staging_buffer.insert(_staging_buffer.end(), payload, payload + payload_len);
            _staging_count++;

            // Push to queue once a full millisecond is staged
            if (_staging_count >= _packets_per_ms) {
                std::lock_guard<std::mutex> lock(_mtx);
                _ready_queue.push_back({_staging_header, _staging_buffer});
                _last_unix_time = hdr->unix_time;
                
                if (_ready_queue.size() > _max_queue_size) {
                    _ready_queue.pop_front();
                }
                
                _staging_buffer.clear();
                _staging_count = 0;
            }
        }
    } // End of while (_is_running)
}


bool ElasticReceiver::get_ms_blocks(uint8_t* out, RFE_Header_t& first_header, size_t num_ms) {
    while (_is_running) {
        {
            std::lock_guard<std::mutex> lock(_mtx);
            if (_ready_queue.size() >= num_ms) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!_is_running) return false;

    std::lock_guard<std::mutex> lock(_mtx);
    for (size_t i = 0; i < num_ms; ++i) {
        MillisecondBlock block = _ready_queue.front();
        if (i == 0) first_header = block.header;
        
        memcpy(out + (i * 8184), block.data.data(), 8184);
        _ready_queue.pop_front();
    }

    return true;
}
