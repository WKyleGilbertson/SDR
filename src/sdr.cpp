#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <thread>
#include <numeric>
#include "version.h"
#include "ElasticReceiver.h"
#include "ChannelProcessor.h"
#include "PCSEngine.hpp"

int main()
{
    SWV V;
    V.Major = MAJOR_VERSION;
    V.Minor = MINOR_VERSION;
    V.Patch = PATCH_VERSION;
    sscanf(CURRENT_HASH, "%x", &V.GitTag);
    strncpy(V.GitCI, CURRENT_HASH, 40);
    V.GitCI[40] = '\0'; // Ensure null-termination
    strncpy(V.BuildDate, CURRENT_DATE, sizeof(V.BuildDate) - 1);
    V.BuildDate[sizeof(V.BuildDate) - 1] = '\0'; // Ensure null-termination
    strncpy(V.Name, APP_NAME, sizeof(V.Name) - 1);
    V.Name[sizeof(V.Name) - 1] = '\0'; // Ensure null-termination

    fprintf(stdout, "%s GitCI:%s %s v%.1d.%.1d.%.1d\n",
            V.Name, V.GitCI, V.BuildDate,
            V.Major, V.Minor, V.Patch);

    try
    {
        ElasticReceiver rx;
        // Connect to the UDP relay
        if (!rx.connect_to_relay("127.0.0.1", 12345))
            return -1;

        PCSEngine pcs(16368000.0);
        // const size_t samples_per_ms = 16368; // 16.368 MHz / 1000
        const size_t samples_per_ms = 8184; // 16.368 MHz / 1000
        bool acq_needed = true;

        // Initialize ChannelProcessor (Assumes 16.368 MHz internally)
        ChannelProcessor chan(16368000.0);

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
                    const size_t SAMPLES_PER_MS = 16368;
                    const size_t FFT_SIZE = 16384;
                    const int NUM_MS = 5;
                    uint32_t anchor = rx.getLastTick();
                    //uint8_t eighth = (anchor % 16368) / 2046;
                    uint32_t absSample = 0;
                    uint16_t phase = anchor % 16368;
                    float absoluteCodePhase = 0.0f;

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
                            absSample = (int)res.peakIndex - (int)phase;
                            while(absSample < 0)
                                absSample += 16368;
                                absSample %= 16368; // Ensure wrap-around
                            absoluteCodePhase = (float)absSample / 16.0f;
                            while (absoluteCodePhase >= 1023.0f)
                                absoluteCodePhase -= 1023.0f; // Wrap around if needed

                            printf("LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f | Carrier %5.2f %6d\n",
                                   //prn, res.snr, res.bin, (float)res.peakIndex / 16, res.phase, res.sampleTick % 16368);
                                   prn, res.snr, res.bin, absoluteCodePhase, res.phase, res.sampleTick % 16368);
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
                            absSample = (int)w_res.peakIndex - (int)phase;
                            while(absSample < 0)
                                absSample += 16368;
                            absSample %= 16368; // Ensure wrap-around
                            absoluteCodePhase = (float)absSample / 16.0f; // Convert to code phase (0-1023.0)
                            if (absoluteCodePhase >= 1023.0f)
                                absoluteCodePhase -= 1023.0f; // Wrap around if needed
                            printf("LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f | Carrier %5.2f %6d\n",
                                   //prn, w_res.snr, w_res.bin, (float)w_res.peakIndex / 16, w_res.phase, w_res.sampleTick % 16368);
                                   prn, w_res.snr, w_res.bin, absoluteCodePhase, w_res.phase, w_res.sampleTick % 16368);
                        }
                    }
                    acq_needed = false;
                }

                // Process the 10ms block (using 0.0 Hz for blind energy detection)
                CorrRes res = chan.process(block.data(), block_size, 0.0);

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
                if (lag > 0.005)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));

                // UI Update every 200ms
                if (std::chrono::duration<double>(now - last_display).count() >= 0.2)
                {
                    double avg_lag = std::accumulate(lag_history.begin(), lag_history.end(), 0.0) / lag_history.size();

                    // Average the magnitude over the display window
                    double display_mag = (mag_count > 0) ? (accumulated_mag / mag_count) : 0;

                    // Output with fixed formatting to prevent text jitter
                    printf("\r[DSP] T+%7.2fs | Mag:%9.0f | Avg Lag:%+8.4fs",
                           session_time, display_mag, avg_lag);
                    fflush(stdout);

                    // Reset display counters
                    last_display = now;
                    accumulated_mag = 0;
                    mag_count = 0;
                }
            }
            else
            {
                // If the buffer is empty, don't spin the CPU at 100%
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
    catch (...)
    {
        std::cerr << "\n[!] Error in SDR_test loop." << std::endl;
    }
    return 0;
}