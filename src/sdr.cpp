#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <thread>
#include <numeric>
#include "version.h"
#include "ElasticReceiver.h"
#include "ChannelProcessor.h"

int main() {
    SWV V;
    V.Major = MAJOR_VERSION; V.Minor = MINOR_VERSION; V.Patch = PATCH_VERSION;
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

    try {
        ElasticReceiver rx;
        // Connect to the UDP relay
        if (!rx.connect_to_relay("127.0.0.1", 12345)) return -1;

        // Initialize ChannelProcessor (Assumes 16.368 MHz internally)
        ChannelProcessor chan(16368000.0);
        
        // 10ms of data (8184 bytes per ms * 10)
        const size_t block_size = 8184 * 10; 
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

        while (true) {
            // Pull data from the ElasticReceiver's Ring Buffer
            if (rx.get_samples(block.data(), block_size)) {
                if (first) { 
                    start_wall = std::chrono::steady_clock::now(); 
                    first = false; 
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
                if (std::abs(lag) > 0.100) {
                    start_wall = now;
                    total_data_time = 0;
                    lag = 0;
                }

                // Smooth out the lag display
                lag_history.push_back(lag);
                if (lag_history.size() > 20) lag_history.erase(lag_history.begin());
                
                // Throttling: If we are processing faster than real-time, breathe
                if (lag > 0.005) std::this_thread::sleep_for(std::chrono::milliseconds(1));

                // UI Update every 200ms
                if (std::chrono::duration<double>(now - last_display).count() >= 0.2) {
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
            } else {
                // If the buffer is empty, don't spin the CPU at 100%
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    } catch (...) { 
        std::cerr << "\n[!] Error in SDR_test loop." << std::endl; 
    }
    return 0;
}