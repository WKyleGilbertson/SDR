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

int main(int argc, char *argv[])
{
    versionInfo v;
    v.printVersion();
    RFE_Header_t meta = {};
    std::vector<ChannelState> activeChannels;
    auto system_start = std::chrono::steady_clock::now();
    uint32_t focusPRN = 131;

    if (argc > 1)
    {
        focusPRN = atoi(argv[1]);
    }

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

        const int tracking_ms = 10; // Process 10 ms at a time for efficiency
        const size_t samples_per_ms = (size_t)(meta.fs_rate / 1000.0);

        while (true)
        {
            // 1. Pull 5ms of data
            if (rx.get_ms_blocks(block.data(), meta, tracking_ms))
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
                        // 1. Reserve memory so the vector doesn't move objects around
                        activeChannels.reserve(results.size());

                        for (const auto &res : results)
                        {
                            // 2. Create the channel and add it to the list
                            activeChannels.emplace_back(res.prn, (double)meta.fs_rate, res, pcs.getSV(res.prn));

                            // 3. NOW it is safe to get the back element because we just added it
                            auto &state = activeChannels.back();

                            if (state.prn == focusPRN)
                            {
                                state.decoder->setFocus(true);
                                printf("  LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f <--- FOCUS\n",
                                       res.prn, res.snr, res.bin, res.codePhase);
                            }
                            else
                            {
                                state.decoder->setFocus(false);
                                printf("  LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n",
                                       res.prn, res.snr, res.bin, res.codePhase);
                            }
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
                    // Track total samples for the Observation timestamp
                    double currentTotalSamples = total_data_time * meta.fs_rate;

                    for (auto &state : activeChannels)
                    {
                        // 1. Tracker does the heavy math (NCOs, Correlations, Bit Sync)
                        CorrRes res = state.processor->process(block.data(), block_size);

                        // 2. Decoder does the logic (Preamble, HOW, TOW, Subframes)
                        // We pass the sample count and the tracker's fine code phase
                        state.decoder->processBits(res.symbols, currentTotalSamples, res.code_phase);

                        // 3. Telemetry (Optional: only print for your focus PRN)
                        if (state.prn == focusPRN)
                        {
                            printf("[PRN %3d] SNR: %5.1f | dF: %7.1f Hz | Phase: %7.2f | \n  ",
                                   state.processor->getPRN() , res.snr, res.dopplerHZ, res.code_phase);
                            /*
                            printf("[PRN %3d] SNR: %4.1f dB | Locked: %s | NavBits: %zu\n",
                                   state.prn,
                                   state.processor->getSNR(),
                                   state.processor->isLocked() ? "YES" : "NO",
                                   */
                            //                                   res.navBits.size());
                            // The decoder's handleWordEnd will already print TOW/Subframe
                            // thanks to the printf inside NavDecoder.cpp
                        }
                    }
                    total_data_time += (double)tracking_ms / 1000.0;
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