#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <thread>
#include <numeric>
#include "L1IFUtil.hpp"
#include "versionInfo.hpp"
#include "ElasticReceiver.h"
#include "AcquisitionMgr.hpp"
#include "ChannelProcessor.h"
#include "PCSEngine.hpp"

int main()
{
    versionInfo v;
    v.printVersion();
    RFE_Header_t meta = {};
    std::vector<ChannelState> activeChannels;
    auto system_start = std::chrono::steady_clock::now();

    try
    {
        ElasticReceiver rx;
        if (!rx.connect_to_relay("127.0.0.1", 12345))
            return -1;

        std::cout << "[*] Waiting for stream telemetry..." << std::endl;
        std::vector<uint8_t> startup_buffer(8184);

        if (!rx.get_ms_blocks(startup_buffer.data(), meta, 1))
        {
            std::cerr << "[!] No data received from relay." << std::endl;
            return -1;
        }

        PCSEngine pcs((float)meta.fs_rate);
        AcquisitionMgr acqMgr(pcs);

        bool acq_needed = true;
        const size_t block_size = 327360;
        std::vector<uint8_t> block(block_size);

        auto start_wall = std::chrono::steady_clock::now();
        double total_data_time = 0;
        bool first = true;

        while (true)
        {
            // 1. Pull 5ms of data
            if (rx.get_ms_blocks(block.data(), meta, 5))
            {
                if (first)
                {
                    start_wall = std::chrono::steady_clock::now();
                    first = false;
                }

                // 2. ACQUISITION PHASE
                if (acq_needed)
                {
                    printf("[*] Starting Acquisition on 5ms block...\n");
                    auto results = acqMgr.run(meta, block.data());

                    if (!results.empty())
                    {
                        activeChannels.clear();
                        for (const auto &res : results)
                        {
                            ChannelState state;
                            state.prn = res.prn;
                            state.result = res;
                            state.sv = pcs.getSV(res.prn);
                            state.processor = std::make_unique<ChannelProcessor>((double)meta.fs_rate, state.result, state.sv);
                            activeChannels.push_back(std::move(state));

                            printf("  LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n",
                                   res.prn, res.snr, res.bin, res.codePhase);
                        }
                        printf("[*] HANDOVER SUCCESS: %zu channels initialized.\n", activeChannels.size());
                        acq_needed = false;
                        rx.jump_to_latest_epoch();
                        continue;
                    }
                    else
                    {
                        printf("[!] No satellites found. Retrying...\n");
                        continue;
                    }
                }

                // 3. TRACKING & DATA DECODING
                if (!activeChannels.empty())
                {
                    int focusPRN = 15; 

                    for (auto &state : activeChannels)
                    {
                        CorrRes res = state.processor->process(block.data(), block_size);

                        if (state.prn == focusPRN)
                        {
                            for (int8_t bitSign : res.navBits)
                            {
                                uint8_t rawBit = (bitSign > 0) ? 1 : 0;

                                // --- PREAMBLE SEARCH ---
                                state.navShiftReg = (state.navShiftReg << 1) | rawBit;
                                uint8_t pattern = (uint8_t)(state.navShiftReg & 0xFF);

                                if (!state.frameSync && (pattern == 0x8B || pattern == 0x74)) 
                                {
                                    state.frameSync = true;
                                    state.inverted = (pattern == 0x74);
                                    state.subframeBitIdx = 0; 
                                    state.lastBitOfPrevWord = 0; 

                                    printf("\n[>>>] SYNC: %02X | Inverted: %s\n", pattern, state.inverted ? "YES" : "NO");

                                    // Manually push the 8 bits of the preamble
                                    uint8_t preamble = 0x8B; 
                                    for(int i=7; i>=0; i--) {
                                        uint8_t b = (preamble >> i) & 1;
                                        printf("%c", (b == 1) ? '#' : '-');
                                        state.subframeBitIdx++;
                                    }
                                    // Use 'continue' to skip the printing block for THIS bit,
                                    // as we just manually handled the first 8 bits of the word.
                                    continue; 
                                }

                                // --- DATA PRINTING ---
                                if (state.frameSync) 
                                {
                                    uint8_t correctedBit = state.inverted ? (rawBit ^ 1) : rawBit;
                                    uint8_t unmaskedBit = (state.subframeBitIdx < 24 && state.lastBitOfPrevWord == 1) 
                                                          ? (correctedBit ^ 1) 
                                                          : correctedBit;

                                    printf("%c", (unmaskedBit == 1) ? '#' : '-');
                                    state.subframeBitIdx++;

                                    if (state.subframeBitIdx % 30 == 0) {
                                        state.lastBitOfPrevWord = correctedBit; 
                                        printf(" [W%d] ", state.subframeBitIdx / 30);
                                    }

                                    if (state.subframeBitIdx >= 300) {
                                        state.subframeBitIdx = 0;
                                        state.frameSync = false; 
                                        printf("\n[END] SUBFRAME -> ");
                                    }
                                }
                                fflush(stdout);
                            } // end bit loop
                        } // end focus check
                    } // end channel loop
                    total_data_time += 0.005;
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        } // end while true
    } // end try
    catch (const std::exception &e)
    {
        std::cerr << "\n[!] Exception: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "\n[!] Unknown Error in SDR_test loop." << std::endl;
    }

    return 0;
}