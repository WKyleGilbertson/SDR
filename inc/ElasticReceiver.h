#pragma once
#include <vector>
#include <mutex>
#include <stdint.h>
#include <winsock2.h>

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

class ElasticReceiver {
public:
    ElasticReceiver(size_t ring_size = 1024 * 1024 * 128);
    ~ElasticReceiver();

    bool connect_to_relay(const char* ip, int port);
    bool get_samples(uint8_t* out, size_t count);

private:
    void ingest_thread();
    
    std::vector<uint8_t> _ring;
    size_t _w_ptr;
    size_t _r_ptr;
    std::mutex _mtx;
    SOCKET _s;
    bool _is_running;

    uint32_t _last_unix_time = 0; // Tracks the roll
    bool _aligned = false;        // Move this to class scope so we can reset it
};