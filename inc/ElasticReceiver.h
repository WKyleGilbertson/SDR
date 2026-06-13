#pragma once
#include <vector>
#include <mutex>
#include <deque>
#include <stdint.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include "L1IFUtil.hpp"

struct MillisecondBlock
{
    RFE_Header_t header;
    std::vector<uint8_t> data; // 8184 bytes
};

struct RingTimingStatus
{
    uint64_t write_index = 0;
    uint64_t oldest_available = 0;
    uint64_t point = 0;
    uint64_t capacity_samples = 0;

    double lag_ms = 0.0;
    double margin_ms = 0.0;
    double ring_ms = 0.0;
};

class ElasticReceiver
{
public:
    ElasticReceiver(size_t max_blocks = 1000);
    ~ElasticReceiver();

    bool connect_to_relay(const char *ip, int port);
    void jump_to_latest_epoch();
    // New method: pulls N milliseconds of data at once
    bool get_ms_blocks(uint8_t *out, RFE_Header_t &first_header, size_t num_ms);
    size_t get_queue_size()
    {
        std::lock_guard<std::mutex> lock(_mtx);
        return _ready_queue.size();
    };

    void unpack_to_ring(const uint8_t *packed,
                        size_t packed_count,
                        uint32_t sample_tick,
                        uint32_t unix_time);

    bool validate_ring_continuity(size_t lookback = 1000);

    uint64_t get_write_index()
    {
        std::lock_guard<std::mutex> lock(_ring_mtx);
        return _write_index;
    };
    bool get_window(
        uint64_t start_index,
        RawSample *&out_ptr,
        unsigned int count,
        std::vector<RawSample> &scratch);
    uint32_t get_last_unix_time()
    {
        std::lock_guard<std::mutex> lock(_mtx);
        return _last_unix_time;
    };

    TimeTrio get_time_trio();
    RingTimingStatus get_timing_status( uint64_t point, size_t samples_per_ms) const;
    bool input_is_complex() const { return _input_is_complex;}
    bool input_mode_checked() const {return _input_mode_checked;}

private:
    void ingest_thread();
    uint16_t _samples_per_ms = 16368;
    uint8_t _packets_per_ms = 8; // 16368 samples/ms / 2046 samples/packet
    uint64_t _base_sample_index = 0;
    uint64_t _global_sample_index = 0;
    uint64_t _last_seq_num = 0;
    // std::deque<MillisecondBlock> _ready_queue; /* ChatGPT removed */
    std::vector<RawSample> _sample_ring;
    mutable std::mutex _ring_mtx;
    uint64_t _write_index = 0;
    size_t _ring_capacity = 0;

    std::deque<MillisecondBlock> _ready_queue;
    std::vector<uint8_t> _staging_buffer;
    RFE_Header_t _staging_header;
    int _staging_count = 0;

    std::mutex _mtx;
    SOCKET _s;
    bool _is_running;
    uint32_t _last_unix_time = 0;
    uint16_t _ms_count = 0; // Which millisecond (0-999)
    uint16_t _last_ms_count = 0;
    uint8_t _ms_frac = 0; // Which fraction (0-7, 03, or 0-1)
    uint8_t _last_ms_frac = 0;
    bool _aligned = false;
    sockaddr_in _relay_addr{};

    size_t _max_queue_size;
    static constexpr size_t RING_DEPTH_MS = 1000;
    bool _input_is_complex = true;
    bool _input_mode_checked = false;
    int _real_votes = 0;
    int _complex_votes = 0;
    int _mode_checks = 0;
    void update_input_mode_detector();
};
