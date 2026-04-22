#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <time.h>
#include "version.h"
#include "ftd2xx.h"

#define CHUNK_SIZE 1024
#define PAYLOAD_SIZE 1023
#define FIFO_SIZE 65536 
#define DEST_PORT "12345"
#define SAMPLE_RATE 16368000 // 16.368 MHz

#pragma pack(push, 1)
typedef struct {
    uint8_t   pkt_type;    // 0x01: Data, 0x02: Cmd
    uint32_t  fs_rate;     // Sample rate (16368000)
    uint32_t  unix_time;
    uint32_t  sample_tick; // Odometer
    uint32_t  seq_num;     // Continuity check
    char      dev_tag[16]; 
    uint16_t  payload_len; // Actual bytes following this header
} RFE_Header_t;
#pragma pack(pop)

struct InternalState {
    unsigned char fifo[FIFO_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    uint32_t max_count; // Track peak buffer usage
    unsigned char hw_tmp[CHUNK_SIZE];
    unsigned char udp_out[PAYLOAD_SIZE + sizeof(RFE_Header_t)];
    char ftdi_sn[16];   // Increased size for safety
    RFE_Header_t hdr;
};

static struct InternalState S;

int main(int argc, char** argv) {
    WSADATA wsaData;
    FT_HANDLE ftH;
    SOCKET sock;
    struct sockaddr_in servaddr;
    SWV V;

    V.Major = MAJOR_VERSION; V.Minor = MINOR_VERSION; V.Patch = PATCH_VERSION;
    sscanf(CURRENT_HASH, "%x", &V.GitTag);
    strncpy(V.GitCI, CURRENT_HASH, 40);
    V.GitCI[40] = '\0'; // Ensure null-termination
    strncpy(V.BuildDate, CURRENT_DATE, sizeof(V.BuildDate) - 1);
    strncpy(V.Name, APP_NAME, sizeof(V.Name) - 1);

      fprintf(stdout, "%s GitCI:%s %s v%.1d.%.1d.%.1d\n",
          V.Name, V.GitCI, V.BuildDate,
          V.Major, V.Minor, V.Patch);

    // Initialize Header Defaults
    memset(&S, 0, sizeof(struct InternalState));
    S.hdr.pkt_type = 0x01;
    S.hdr.fs_rate = SAMPLE_RATE;
    S.hdr.payload_len = PAYLOAD_SIZE;

    const char* target_host = (argc > 1) ? *(argv + 1) : "P2P.local";

    WSAStartup(MAKEWORD(2, 2), &wsaData);
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (FT_OpenEx((void*)"USB<->GPS A", FT_OPEN_BY_DESCRIPTION, &ftH) != FT_OK) {
        printf("[!] Error: Could not open FTDI device.\n");
        return 1;
    }

    // Get Serial for dev_tag
    if (FT_GetDeviceInfo(ftH, NULL, NULL, S.ftdi_sn, NULL, NULL) == FT_OK) {
        printf("[*] FTDI Serial: %.15s\n", S.ftdi_sn);
        memset(S.hdr.dev_tag, 0, 16);
        strncpy(S.hdr.dev_tag, S.ftdi_sn, 15);
    }

    FT_SetTimeouts(ftH, 100, 100);
    FT_SetLatencyTimer(ftH, 2);
    FT_SetUSBParameters(ftH, 65536, 65536);
    FT_Purge(ftH, FT_PURGE_RX | FT_PURGE_TX);

    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    if (getaddrinfo(target_host, DEST_PORT, &hints, &res) != 0) {
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(12345);
        inet_pton(AF_INET, "127.0.0.1", &servaddr.sin_addr);
    } else {
        memcpy(&servaddr, res->ai_addr, res->ai_addrlen);
        freeaddrinfo(res);
    }

    uint32_t absolute_tick = 0;
    uint32_t bytes_since_print = 0;
    uint64_t total_bytes = 0;
    time_t start_t = time(NULL);
    time_t last_print_t = start_t;

    printf("[*] Streaming to %s | Fs: %.3f MHz\n", target_host, (double)SAMPLE_RATE/1e6);

    while (1) {
        DWORD bQ = 0, bR = 0;
        FT_GetQueueStatus(ftH, &bQ);

        if (bQ >= CHUNK_SIZE) {
            if (FT_Read(ftH, S.hw_tmp, CHUNK_SIZE, &bR) == FT_OK) {
                for (DWORD i = 0; i < bR; i++) {
                    S.fifo[S.head] = S.hw_tmp[i];
                    S.head = (S.head + 1) % FIFO_SIZE;
                    if (S.count < FIFO_SIZE) S.count++;
                    else S.tail = (S.tail + 1) % FIFO_SIZE;
                }
                
                // Track max buffer usage
                if (S.count > S.max_count) S.max_count = S.count;

                while (S.count >= PAYLOAD_SIZE) {
                    S.hdr.unix_time = (uint32_t)time(NULL);
                    S.hdr.sample_tick = absolute_tick;
                    
                    // Copy header
                    memcpy(S.udp_out, &S.hdr, sizeof(RFE_Header_t));

                    // Copy payload
                    for (int i = 0; i < PAYLOAD_SIZE; i++) {
                        S.udp_out[sizeof(RFE_Header_t) + i] = S.fifo[S.tail];
                        S.tail = (S.tail + 1) % FIFO_SIZE;
                        S.count--;
                    }

                    sendto(sock, (const char*)S.udp_out, PAYLOAD_SIZE + sizeof(RFE_Header_t), 0, (struct sockaddr*)&servaddr, sizeof(servaddr));

                    absolute_tick += (PAYLOAD_SIZE * 2); // 2 samples per byte
                    bytes_since_print += PAYLOAD_SIZE;
                    total_bytes += PAYLOAD_SIZE;
                    S.hdr.seq_num++;

                    // Print status every ~1 second (8.184MB at 16.368Msps)
                    if (bytes_since_print >= 8184000) {
                        time_t now = time(NULL);
                        double elapsed = difftime(now, start_t);
                        double delta_t = difftime(now, last_print_t);
                        if (delta_t < 0.1) delta_t = 1.0; // Prevent div by zero

                        double actual_mbps = (bytes_since_print * 8.0) / (delta_t * 1000000.0);
                        double expected_mbps = (SAMPLE_RATE * 4.0) / 1000000.0; // 4 bits/sample

                        printf("\r[UP:%02d:%02d:%02d] Sent:%6.1fMB | Speed:%5.2fMbps (%4.1f%%) | Buf:%2d%%", 
                            (int)elapsed/3600, ((int)elapsed%3600)/60, (int)elapsed%60, 
                            (double)total_bytes/1048576.0, 
                            actual_mbps, 
                            (actual_mbps / expected_mbps) * 100.0,
                            (S.max_count * 100) / FIFO_SIZE);
                        
                        fflush(stdout);
                        bytes_since_print = 0;
                        S.max_count = 0; // Reset peak tracker
                        last_print_t = now;
                    }
                }
            }
        }
    }
    return 0;
}