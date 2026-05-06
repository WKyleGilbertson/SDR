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
    std::vector<AcqResult> active_channels;
    std::vector<ChannelState> activeChannels;
    // AcqResult target = {};
    auto system_start = std::chrono::steady_clock::now();

    try
    {
        ElasticReceiver rx;
        // Connect to the UDP relay
        if (!rx.connect_to_relay("127.0.0.1", 12345))
            return -1;

        // --- WAIT FOR FIRST PACKET ---
        std::cout << "[*] Waiting for stream telemetry..." << std::endl;

        std::vector<uint8_t> startup_buffer(8184);

        if (!rx.get_ms_blocks(startup_buffer.data(), meta, 1))
        {
            std::cerr << "[!] No data received from relay. Check if it's running." << std::endl;
            return -1;
        }
        printf("[*] Connected to Device: %s | FS: %u Hz | Last Tick: %u\n",
               meta.dev_tag, meta.fs_rate, meta.sample_tick);
        // -----------------------------

        PCSEngine pcs((float)meta.fs_rate);
        AcquisitionMgr acqMgr(pcs);

        bool acq_needed = true;

        std::unique_ptr<ChannelProcessor> chan;
        //        printf(" [NEW CHAN PTR: %p] ", (void *)chan.get());

        // 10ms of data (8184 bytes per ms * 10)
        // const size_t block_size = 8184 * 10;
        const size_t block_size = 327360;
        std::vector<uint8_t> block(block_size);

        auto start_wall = std::chrono::steady_clock::now();
        double total_data_time = 0, session_time = 0;
        bool first = true;

        std::vector<double> lag_history;
        auto last_display = std::chrono::steady_clock::now();

        // Variables for Integrated Magnitude display
        double accumulated_mag = 0;
        int mag_count = 0;

        std::cout << "[*] SDR Staging: Processing..." << std::endl;

        if (first)
        {
            auto sync_end = std::chrono::steady_clock::now();
            double sync_duration = std::chrono::duration<double>(sync_end - system_start).count();

            std::cout << "[*] Total time to telemetry sync: " << sync_duration << "s" << std::endl;

            start_wall = sync_end; // Existing lag logic
            first = false;
        }

        while (true)
        {
            // Pull data from the ElasticReceiver's Ring Buffer
            // if (rx.get_ms_blocks(block.data(), meta, 10))
            if (rx.get_ms_blocks(block.data(), meta, 5))
            {
                if (first)
                {
                    start_wall = std::chrono::steady_clock::now();
                    first = false;
                }
                if (acq_needed)
                {
                    auto start_acq = std::chrono::steady_clock::now();
                    TimeTrio tme3 = rx.get_time_trio();
                    std::string block_tag = get_iso8601_timestamp(tme3.unixSecond, tme3.msCount);
                    fprintf(stdout, "[*] Current Buffer Insertion Time: %s\n",
                            block_tag.c_str());
                    TimeTrio acqTime = get_timeData(meta.unix_time, meta.sample_tick, meta.fs_rate);
                    std::string acq_tag = get_iso8601_timestamp(acqTime.unixSecond, acqTime.msCount);
                    fprintf(stdout, "[*] Block Time: %s Unix: %u, Tick: %u\n",
                            acq_tag.c_str(), acqTime.unixSecond, meta.sample_tick);

                    // 1. Run the acquisition
                    // auto results = acqMgr.run(meta, block.data());
                    auto results = acqMgr.run(meta, block.data());

                    auto end_acq = std::chrono::steady_clock::now();
                    double acq_duration = std::chrono::duration<double>(end_acq - start_acq).count();
                    if (!results.empty())
                    {
                        activeChannels.clear();
                        for (const auto &res : results)
                        {
                            ChannelState state;
                            state.prn = res.prn;
                            state.result = res;
                            state.sv = pcs.getSV(res.prn);
                            state.processor = std::make_unique<ChannelProcessor>(
                                (double)meta.fs_rate, state.result, state.sv);

                            activeChannels.push_back(std::move(state));

                            printf("LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n",
                                   res.prn, res.snr, res.bin, res.codePhase);
                        }

                        printf("[*] HANDOVER SUCCESS: %zu channels initialized.\n", activeChannels.size());
                        acq_needed = false;
                        rx.jump_to_latest_epoch();
                        continue;
                    }

                } // End acq_needed

                if (!activeChannels.empty())
                {
                    // int focusPRN = 135; // Set target here
                     int focusPRN = 131; // Set target here
                    //int focusPRN = 29; // Set target here
                    for (auto &state : activeChannels)
                    {
                        // 1. Process EVERY channel so they stay locked
                        CorrRes res = state.processor->process(block.data(), block_size);

                        // 2. ONLY run UI logic for the focus PRN
                        if (state.prn == focusPRN)
                        {
                            std::string bits = "";
                            for (int8_t s : res.symbols)
                            {
                                bits += (s > 0) ? "#" : "-";
                            }

                            double current_mag = std::sqrt(res.Pi * res.Pi + res.Pq * res.Pq);
                            accumulated_mag += current_mag;
                            mag_count++;
                            lag_history.push_back((res.code_phase - state.handoverPhase) / 1023000.0);

                            auto now = std::chrono::steady_clock::now();
                            if (std::chrono::duration<double>(now - last_display).count() >= 0.2)
                            {
                                double avg_drift = lag_history.empty() ? 0.0 : std::accumulate(lag_history.begin(), lag_history.end(), 0.0) / lag_history.size();

                                // printf("\r[TRK] PRN %3d |Code: %8.3f | I: %6.0f | Q: %6.0f | Mag: %5.0f | D: %3.1es",
                                //        state.prn, res.code_phase, res.Pi, res.Pq, accumulated_mag, avg_drift);
                                //  Updated Printf to show the bit pattern at the end
                                printf("\r[TRK] PRN %3d |Code: %8.3f | I: %6.0f | Q: %6.0f | D: %3.1es | Bits: %s",
                                       state.prn, res.code_phase, res.Pi, res.Pq, avg_drift, bits.c_str());
                                fflush(stdout);

                                last_display = now;
                                accumulated_mag = 0;
                                mag_count = 0;
                                if (lag_history.size() > 50)
                                    lag_history.clear();
                            }
                        }
                    }

                    // ... rest of throttling logic ...
                    // Update time tracking (only once per 10ms block, not per channel!)
                    total_data_time += 0.010;
                    session_time += 0.010;

                    // Latency and Throttling logic remains here...
                    auto now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now - start_wall).count();
                    double lag = total_data_time - elapsed;
                    // ... (keep your existing lag/sleep logic) ...
                }
            } // end if (rx.get_samples)
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } // end while (true)
    } // end try
    catch (...)
    {
        std::cerr << "\n[!] Error in SDR_test loop." << std::endl;
    }
    return 0;
} // end main