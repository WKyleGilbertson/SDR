#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <thread>
#include <numeric>
#include "versionInfo.hpp"
#include "ElasticReceiver.h"
#include "ChannelProcessor.h"
#include "PCSEngine.hpp"

int main()
{
    versionInfo v;
    v.printVersion();
    RFE_Header_t meta = {};
    std::vector<AcqResult> active_channels;
    AcqResult target = {};

    try
    {
        ElasticReceiver rx;
        // Connect to the UDP relay
        if (!rx.connect_to_relay("127.0.0.1", 12345))
            return -1;

        // --- WAIT FOR FIRST PACKET ---
        std::cout << "[*] Waiting for stream telemetry..." << std::endl;
        int timeout = 0;
        while (rx.get_latest_header().fs_rate == 0 && timeout < 500)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            timeout++;
        }

        if (timeout >= 500)
        {
            std::cerr << "[!] No data received from relay. Check if it's running." << std::endl;
            return -1;
        }

        else
        {
            meta = rx.get_latest_header();
            printf("[*] Connected to Device: %s | FS: %u Hz | Last Tick: %u\n",
                   meta.dev_tag, meta.fs_rate, meta.sample_tick);
        }
        // -----------------------------

        PCSEngine pcs((float)meta.fs_rate);
        bool acq_needed = true;

        std::unique_ptr<ChannelProcessor> chan;
        // std::make_unique<ChannelProcessor> chan;
        printf(" [NEW CHAN PTR: %p] ", (void *)chan.get());

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

        while (true)
        {
            // Pull data from the ElasticReceiver's Ring Buffer
            if (rx.get_samples(block.data(), block_size))
            {
                if (first)
                {
                    start_wall = std::chrono::steady_clock::now();
                    first = false;
                }
                // --- NEW: Acquisition Logic ---

                if (acq_needed)
                {
                    //     std::make_unique<ChannelProcessor> chan;
                    //     printf(" [NEW CHAN PTR: %p] ", (void*)chan.get());
                    const size_t SAMPLES_PER_MS = meta.fs_rate / 1000;   // 16386
                    const size_t SAMPLES_PER_PKT = meta.payload_len * 2; // FNHN
                    const size_t FFT_SIZE = 16384;                       // 2^14, 16 more than 16386
                    const int NUM_MS = 5;
                    uint32_t anchor = rx.getLastTick(); // Sample in the second
                    uint8_t eighth = (anchor % SAMPLES_PER_MS) / SAMPLES_PER_PKT;
                    float absoluteCodePhase = 0.0f;         // ms relative
                    float relativeCodePhase = 0.0f;         // pkt relative
                    const float phaseInterval = 1023.0 / 8; // 8 pkts per ms
                    float offset = eighth * phaseInterval;  // pkt->ms offset

                    // 1. Prepare the buffer: size MUST be a multiple of 16384 (81920 total)
                    // This replicates: std::vector<kiss_fft_cpx> data(16384 * config.numMs);
                    std::vector<kiss_fft_cpx> aligned_data(FFT_SIZE * NUM_MS, {0, 0});
                    uint8_t *raw_ptr = block.data();

                    std::cout << "\n[*] Unpacking 5ms (MAX2769 Split-Nibble) with Alignment..." << std::endl;

                    // 2. Unpack with 16-sample padding per ms
                    for (int ms = 0; ms < NUM_MS; ms++)
                    {
                        uint8_t *ms_source = raw_ptr + (ms * 8184); // 8184 bytes per ms
                        kiss_fft_cpx *ms_dest = &aligned_data[ms * FFT_SIZE];

                        for (size_t i = 0; i < 8184; ++i)
                        {
                            unpackL1IF(ms_source[i], ms_dest[2 * i], ms_dest[2 * i + 1]);
                        }
                        // Zero-padding (samples 16368-16383) is already there from vector init
                    }

                    // 3. Search using the PCSEngine logic from pcs.cpp
                    for (int prn = 1; prn <= 32; prn++)
                    {
                        // The engine sees 81920 samples and knows to integrate 5ms
                        AcqResult res = pcs.search(prn, aligned_data, 4.092e6f,
                                                   20, 500.0f, anchor);

                        if (res.snr > 9.0)
                        {
                            relativeCodePhase = (float)((res.peakIndex % 16368) / 16.0f);
                            absoluteCodePhase = relativeCodePhase - offset;
                            while (absoluteCodePhase < 0.0f)
                                absoluteCodePhase += 1023.0f; // Wrap around if needed
                            res.codePhase = absoluteCodePhase;
                            active_channels.push_back(res);

                            printf("LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f | Carrier %5.2f %6d\n",
                                   // prn, res.snr, res.bin, (float)res.peakIndex / 16, res.phase, res.sampleTick % 16368);
                                   prn, res.snr, res.bin, res.codePhase, res.phase, res.sampleTick % 16368);
                        }
                    }
                    // Define the WAAS PRNs we want to hunt
                    std::vector<int> waas_prns = {131, 133, 135};

                    for (int prn : waas_prns)
                    {
                        // We use a slightly wider Doppler bin or smaller step if needed,
                        // but 500Hz steps are usually fine for acquisition.
                        AcqResult w_res = pcs.search(prn, aligned_data, 4.092e6f,
                                                     20, 500.0f, anchor);

                        if (w_res.snr > 8.0f)
                        { // WAAS usually has a decent signal
                            relativeCodePhase = (float)((w_res.peakIndex % 16368) / 16.0f);
                            absoluteCodePhase = relativeCodePhase - offset;
                            while (absoluteCodePhase < 0.0f)
                                absoluteCodePhase += 1023.0f; // Wrap around if needed
                            w_res.codePhase = absoluteCodePhase;
                            active_channels.push_back(w_res);

                            printf("LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f | Carrier %5.2f %6d\n",
                                   // prn, w_res.snr, w_res.bin, (float)w_res.peakIndex / 16, w_res.phase, w_res.sampleTick % 16368);
                                   prn, w_res.snr, w_res.bin, w_res.codePhase, w_res.phase, w_res.sampleTick % 16368);
                        }
                    }
                    if (acq_needed && !active_channels.empty())
                    {
                        AcqResult selected_sat = active_channels[0]; // Fix: pick the first one

                        /*    bool found_131 = false;

                            for (const auto &res : active_channels)
                            {
                                if (res.prn == 131)
                                {
                                    selected_sat = res;
                                    found_131 = true;
                                    break;
                                }
                            }

                            if (found_131)
                            {
                                printf("\n[*] High-Priority Lock: PRN 131 selected.\n");
                            }
                            else
                            {
                                printf("\n[*] PRN 131 not found. Tracking PRN %d.\n", selected_sat.prn);
                            } */

                        // Re-initialize the processor with the locked satellite
                        chan = std::make_unique<ChannelProcessor>((double)meta.fs_rate, selected_sat);
                        printf("\n[*] HANDOVER SUCCESS: New Chan at %p tracking PRN %d\n",
                               (void *)chan.get(), selected_sat.prn);
                        acq_needed = false;
                        continue; // Potentially optional....
                    }
                } // End of (acq_needed)
                if (chan)
                {
                    // Process the 10ms block (using 0.0 Hz for blind energy detection)
                    //printf(" [PTR: %p] ", (void *)chan.get());
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
                        //double current_code = res.code_phase;
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