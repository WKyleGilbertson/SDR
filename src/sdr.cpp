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
    AcqResult target = {};
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
            if (rx.get_ms_blocks(block.data(), meta, 10))
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
                        // 2. Print the list of found satellites to match your desired output
                        for (const auto &res : results)
                        {
                            printf("LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f | Carrier %5.2f %u\n",
                                   res.prn, res.snr, res.bin, res.codePhase, res.phase, res.sampleTick % 16368);
                        }
                        printf("[*] Acquisition Duration: %.2f seconds\n", acq_duration);

                        rx.jump_to_latest_epoch();

                        // 3. Selection Logic: Default to the first satellite, but look for PRN 131
                        AcqResult selected_sat = results[0];
                        bool found_priority = false;
                        for (const auto &res : results)
                        {
                            if (res.prn == 131)
                            {
                                selected_sat = res;
                                found_priority = true;
                                break;
                            }
                        }

                        if (found_priority)
                        {
                            printf("\n[*] High-Priority Lock: PRN 131 selected.\n");
                        }

                        // 4. Handover: Create the tracker and flip the flag
                        chan = std::make_unique<ChannelProcessor>((double)meta.fs_rate, selected_sat);

                        printf("[*] HANDOVER SUCCESS: New Chan at %p tracking PRN %d\n",
                               (void *)chan.get(), selected_sat.prn);

                        acq_needed = false;
                        continue; // Skip the rest of this loop to start tracking immediately
                    }
                }

                if (chan)
                {
                    // Process the 10ms block (using 0.0 Hz for blind energy detection)
                    // printf(" [PTR: %p] ", (void *)chan.get());
                    CorrRes res = chan->process(block.data(), block_size);

                    // Update time tracking
                    total_data_time += 0.010;
                    session_time += 0.010;

                    // Calculate current Magnitude and add to integrator
                    double current_mag = std::sqrt(res.i_val * res.i_val + res.q_val * res.q_val);
                    accumulated_mag += current_mag;
                    mag_count++;

                    // Latency Calculation
                    auto now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now - start_wall).count();
                    double lag = total_data_time - elapsed;

                    // Self-Correcting Sync: Reset if drift > 100ms
                    if (std::abs(lag) > 0.100)
                    {
                        start_wall = now;
                        total_data_time = 0;
                        lag = 0;
                    }

                    // Smooth out the lag display
                    lag_history.push_back(lag);
                    if (lag_history.size() > 20)
                        lag_history.erase(lag_history.begin());

                    // Throttling: If we are processing faster than real-time, breathe
                    if (lag > 0.005) // This said > 0.005 before, but that would only sleep if we were lagging behind, not if we were ahead.
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));

                    // UI Update every 200ms
                    if (std::chrono::duration<double>(now - last_display).count() >= 0.2)
                    {
                        double avg_lag = std::accumulate(lag_history.begin(), lag_history.end(), 0.0) / lag_history.size();

                        // Average the magnitude over the display window
                        double display_mag = (mag_count > 0) ? (accumulated_mag / mag_count) : 0;
                        // Pull live tracking data from the channel object
                        // Assuming your ChannelProcessor has these getters:
                        // double current_code = chan->getCodePhase();
                        double current_code = res.current_code_phase;
                        // double current_code = res.code_phase;
                        double carrier_phase = atan2(res.q_val, res.i_val);
                        // Enhanced Output
                        printf("\r[TRK] PRN %3d | Carr: %+4.1f Hz | Code: %8.3f | Mag: %5.0f | Lag: %+7.4fs   ",
                               // selected_sat.prn, 0.1, current_code, display_mag, avg_lag);
                               chan->getPRN(), carrier_phase, current_code, display_mag, avg_lag);
                        // Output with fixed formatting to prevent text jitter
                        //                        printf("\r[DSP] T+%7.2fs | Mag:%9.0f | Avg Lag:%+8.4fs",
                        //                               session_time, display_mag, avg_lag);
                        fflush(stdout);

                        // Reset display counters
                        last_display = now;
                        accumulated_mag = 0;
                        mag_count = 0;
                    } // end if (UI Update)
                } // end if (chan)
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