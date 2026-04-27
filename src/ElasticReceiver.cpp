#include "ElasticReceiver.h"
#include <iostream>
#include <thread>
#include <vector>
#include <ctime>
#include <cstdio>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

ElasticReceiver::ElasticReceiver(size_t ring_size)
    : _ring(ring_size), _w_ptr(0), _r_ptr(0), _s(INVALID_SOCKET), _is_running(false), _last_unix_time(0), _aligned(false) {}

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
    std::vector<uint8_t> tmp(4096);
    const int H_SIZE = sizeof(RFE_Header_t);

    while (_is_running)
    {
        sockaddr_in src{};
        int addr_len = sizeof(src);
        int len = recvfrom(_s, (char *)tmp.data(), (int)tmp.size(), 0, (struct sockaddr *)&src, &addr_len);

        if (len >= H_SIZE)
        {
            RFE_Header_t *hdr = (RFE_Header_t *)tmp.data();

            // Check for Unix second roll to align our ring buffer's start (0) to a 1-second epoch
            // Inside ingest_thread, replace the Unix Second Roll block:

            /**/
            if (hdr->unix_time > _last_unix_time)
            {
                if (!_aligned && _last_unix_time != 0)
                {
                    std::lock_guard<std::mutex> lock(_mtx);

                    uint32_t phase = hdr->sample_tick % 16368;
                    uint32_t ms_val = 0;
                    ms_val = (int)((hdr->sample_tick % 16368000) / 16368);

                    _w_ptr = phase;
                    _r_ptr = 0;
                    _aligned = true;
                    _last_sample_tick = hdr->sample_tick;

                    struct tm gmt_info;
                    __time64_t t_val = (__time64_t)hdr->unix_time;

                    if (_gmtime64_s(&gmt_info, &t_val) == 0)
                    {
                        auto print2 = [](int v)
                        {
                            if (v < 10)
                                std::cout << '0';
                            std::cout << v;
                        };

                        std::cout << "\n[+] ZERO CROSS LOCKED: " << (gmt_info.tm_year + 1900) << "-";
                        print2(gmt_info.tm_mon + 1);
                        std::cout << "-";
                        print2(gmt_info.tm_mday);
                        std::cout << "T";
                        print2(gmt_info.tm_hour);
                        std::cout << ":";
                        print2(gmt_info.tm_min);
                        std::cout << ":";
                        print2(gmt_info.tm_sec);
                        std::cout << ".";

                        if (ms_val < 100)
                            std::cout << '0';
                        if (ms_val < 10)
                            std::cout << '0';
                        std::cout << ms_val << "Z | Phase " << phase << std::endl;
                    }
                    else
                    {
                        std::cout << "\n[+] ZERO CROSS LOCKED: [Time Err] | Phase " << phase << std::endl;
                    }
                }
                _last_unix_time = hdr->unix_time;
            }
            /**/
            if (_aligned)
            {
                size_t p_len = (size_t)(len - H_SIZE);
                std::lock_guard<std::mutex> lock(_mtx);
                size_t space = _ring.size() - _w_ptr;
                if (p_len <= space)
                {
                    memcpy(_ring.data() + _w_ptr, tmp.data() + H_SIZE, p_len);
                    _w_ptr = (_w_ptr + p_len) % _ring.size();
                }
                else
                {
                    memcpy(_ring.data() + _w_ptr, tmp.data() + H_SIZE, space);
                    memcpy(_ring.data(), tmp.data() + H_SIZE + space, p_len - space);
                    _w_ptr = p_len - space;
                }
            }
        }
    }
}

bool ElasticReceiver::get_samples(uint8_t *out, size_t count)
{
    while (_is_running)
    {
        if (_aligned)
        {
            size_t avail;
            {
                std::lock_guard<std::mutex> lock(_mtx);
                avail = (_w_ptr >= _r_ptr) ? (_w_ptr - _r_ptr) : (_ring.size() - _r_ptr + _w_ptr);
            }
            if (avail >= count)
                break;
        }
        // If we aren't aligned or don't have enough data, yield.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!_is_running)
        return false;

    std::lock_guard<std::mutex> lock(_mtx);
    size_t space = _ring.size() - _r_ptr;
    if (count <= space)
    {
        memcpy(out, _ring.data() + _r_ptr, count);
        _r_ptr = (_r_ptr + count) % _ring.size();
    }
    else
    {
        memcpy(out, _ring.data() + _r_ptr, space);
        memcpy(out + space, _ring.data(), count - space);
        _r_ptr = count - space;
    }
    return true;
}