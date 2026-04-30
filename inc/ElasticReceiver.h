#pragma once
#include <vector>
#include <mutex>
#include <deque>
#include <stdint.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma pack(push, 1)
struct RFE_Header_t {
    uint8_t pkt_type;
    uint32_t fs_rate;
    uint32_t unix_time;
    uint32_t sample_tick;
    uint32_t seq_num;
    char dev_tag[16];
    uint16_t payload_len;
};
#pragma pack(pop)

struct MillisecondBlock {
    RFE_Header_t header;
    std::vector<uint8_t> data; // 8184 bytes
};

class ElasticReceiver {
public:
    ElasticReceiver(size_t max_blocks = 1000);
    ~ElasticReceiver();

    bool connect_to_relay(const char* ip, int port);
    
    // New method: pulls N milliseconds of data at once
    bool get_ms_blocks(uint8_t* out, RFE_Header_t& first_header, size_t num_ms);
    size_t get_queue_size() {
        std::lock_guard<std::mutex> lock(_mtx);
        return _ready_queue.size();
    };
    uint32_t get_last_unix_time() {
        std::lock_guard<std::mutex> lock(_mtx);
        return _last_unix_time;
    };

private:
    void ingest_thread();
    uint16_t _samples_per_ms = 16368;
    uint8_t _packets_per_ms = 8; // 16368 samples/ms / 2046 samples/packet

    std::deque<MillisecondBlock> _ready_queue;
    std::vector<uint8_t> _staging_buffer;
    RFE_Header_t _staging_header;
    int _staging_count = 0;

    std::mutex _mtx;
    SOCKET _s;
    bool _is_running;
    uint32_t _last_unix_time = 0;
    bool _aligned = false;
    sockaddr_in _relay_addr{};
    
    size_t _max_queue_size;
};
