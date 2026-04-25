#include "ElasticReceiver.h"
#include <iostream>
#include <thread>
#include <vector>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

ElasticReceiver::ElasticReceiver(size_t ring_size) 
    : _ring(ring_size), _w_ptr(0), _r_ptr(0), _s(INVALID_SOCKET), _is_running(false), _last_unix_time(0), _aligned(false) {}

ElasticReceiver::~ElasticReceiver() {
    _is_running = false;
    if (_s != INVALID_SOCKET) closesocket(_s);
    WSACleanup();
}

bool ElasticReceiver::connect_to_relay(const char* ip, int port) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;

    _s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_s == INVALID_SOCKET) return false;

    u_long mode = 1;
    ioctlsocket(_s, FIONBIO, &mode);

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    inet_pton(AF_INET, ip, &dest.sin_addr);

    uint32_t JOIN_CMD = 0x4A4F494E;
    sendto(_s, (const char*)&JOIN_CMD, 4, 0, (struct sockaddr*)&dest, sizeof(dest));

    std::vector<char> flush_buffer(4096);
    int flushed = 0;
    while (recv(_s, flush_buffer.data(), (int)flush_buffer.size(), 0) > 0) { 
        flushed++; 
    }
    std::cout << "[*] Flushed " << flushed << " stale packets." << std::endl;

    mode = 0; 
    ioctlsocket(_s, FIONBIO, &mode);

    _is_running = true;
    std::thread t(&ElasticReceiver::ingest_thread, this);
    t.detach();
    return true;
}

void ElasticReceiver::ingest_thread() {
    std::vector<uint8_t> tmp(4096);
    const int H_SIZE = sizeof(RFE_Header_t);

    while (_is_running) {
        sockaddr_in src{};
        int addr_len = sizeof(src);
        int len = recvfrom(_s, (char*)tmp.data(), (int)tmp.size(), 0, (struct sockaddr*)&src, &addr_len);

        if (len >= H_SIZE) {
            RFE_Header_t* hdr = (RFE_Header_t*)tmp.data();

            // Check for Unix second roll
            if (hdr->unix_time > _last_unix_time) {
                // Only perform hard reset if we are hunting for alignment
                if (!_aligned && (hdr->sample_tick % 16368 == 0)) {
                    std::lock_guard<std::mutex> lock(_mtx);
                    _w_ptr = 0;
                    _r_ptr = 0; 
                    _aligned = true;
                    printf("\n[+] EPOCH LOCKED: Unix %u | Tick %u | Source: %.16s\n", 
                           hdr->unix_time, hdr->sample_tick, hdr->dev_tag);
                }
                _last_unix_time = hdr->unix_time;
            }

            // Only fill the ring if we have an established epoch
            if (_aligned) {
                size_t p_len = (size_t)(len - H_SIZE);
                if (p_len > 0) {
                    std::lock_guard<std::mutex> lock(_mtx);
                    size_t space = _ring.size() - _w_ptr;
                    if (p_len <= space) {
                        memcpy(_ring.data() + _w_ptr, tmp.data() + H_SIZE, p_len);
                        _w_ptr = (_w_ptr + p_len) % _ring.size();
                    } else {
                        memcpy(_ring.data() + _w_ptr, tmp.data() + H_SIZE, space);
                        memcpy(_ring.data(), tmp.data() + H_SIZE + space, p_len - space);
                        _w_ptr = p_len - space;
                    }
                }
            }
        }
    }
}

bool ElasticReceiver::get_samples(uint8_t* out, size_t count) {
    while (_is_running) {
        // If not aligned, don't even look at the pointers, just wait
        if (!_aligned) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        size_t avail;
        {
            std::lock_guard<std::mutex> lock(_mtx);
            avail = (_w_ptr >= _r_ptr) ? (_w_ptr - _r_ptr) : (_ring.size() - _r_ptr + _w_ptr);
        }
        
        if (avail >= count) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!_is_running) return false;

    std::lock_guard<std::mutex> lock(_mtx);
    size_t space = _ring.size() - _r_ptr;
    if (count <= space) {
        memcpy(out, _ring.data() + _r_ptr, count);
        _r_ptr = (_r_ptr + count) % _ring.size();
    } else {
        memcpy(out, _ring.data() + _r_ptr, space);
        memcpy(out + space, _ring.data(), count - space);
        _r_ptr = count - space;
    }
    return true;
}