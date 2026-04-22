#include "ElasticReceiver.h"
#include <iostream>
#include <thread>
#include <vector>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

ElasticReceiver::ElasticReceiver(size_t ring_size) 
    : _ring(ring_size), _w_ptr(0), _r_ptr(0), _s(INVALID_SOCKET), _is_running(false) {}

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

    // Using a vector here to be 100% explicit for the compiler
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
    bool aligned = false;
    const int H_SIZE = sizeof(RFE_Header_t);

    while (_is_running) {
        sockaddr_in src{};
        int addr_len = sizeof(src);
        int len = recvfrom(_s, (char*)tmp.data(), (int)tmp.size(), 0, (struct sockaddr*)&src, &addr_len);

        if (len >= H_SIZE) {
            RFE_Header_t* hdr = (RFE_Header_t*)tmp.data();
            if (!aligned) {
                if ((hdr->pkt_type == 1 || hdr->pkt_type == 49) && (hdr->sample_tick % 16368 == 0)) {
                    aligned = true;
                    printf("\n[+] LOCKED: Phase 0 | Source: %.16s\n", hdr->dev_tag);
                } else continue;
            }

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

bool ElasticReceiver::get_samples(uint8_t* out, size_t count) {
    while (_is_running) {
        size_t avail;
        {
            std::lock_guard<std::mutex> lock(_mtx);
            avail = (_w_ptr >= _r_ptr) ? (_w_ptr - _r_ptr) : (_ring.size() - _r_ptr + _w_ptr);
        }
        if (avail >= count) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

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