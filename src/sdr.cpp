#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <thread>
#include <numeric>
#include <cstdio>
#include <cstdint>
#include <list>
#include "L1IFUtil.hpp"
#include "versionInfo.hpp"
#include "ElasticReceiver.h"
#include "AcquisitionMgr.hpp"
#include "ChannelProcessor.h"
#include "PCSEngine.hpp"
#include "NavDecoder.h"

struct ChannelState {
    int prn;
    AcqResult result; // Metadata from PCS (PRN, CodePhase, Bin)
    G2INIT sv; // The Gold Code replica (bits)
    std::unique_ptr<ChannelProcessor> processor; // The active tracker
    std::unique_ptr<NavDecoder> decoder;
    std::vector<uint8_t> residualSamples; // Residual queue preserving across handovers

    bool isActive() const { return processor != nullptr; }

    ChannelState(int p, double fs, const AcqResult &res, G2INIT s) : prn(p), result(res), sv(s) {
        processor = std::make_unique<ChannelProcessor>(fs, result, s);
        decoder = std::make_unique<NavDecoder>(p);
    }

    // Move semantics support for vector allocation
    ChannelState(ChannelState &&) noexcept = default;
    ChannelState &operator=(ChannelState &&) noexcept = default;

    // Explicitly delete copy constructor assignments
    ChannelState(const ChannelState &) = delete;
    ChannelState &operator=(const ChannelState &) = delete;
};

int main(int argc, char *argv[]) {
    FILE *out = fopen("output.bin", "wb");
    if (!out) {
        std::cerr << "[!] Failed to open output file." << std::endl;
        return -1;
    }

    versionInfo v;
    v.printVersion();

    RFE_Header_t meta = {};
    TimeTrio t3;
    //std::vector<ChannelState> activeChannels;
    std::list<ChannelState> activeChannels;
    auto system_start = std::chrono::steady_clock::now();

    uint32_t focusPRN = 131;
    if (argc > 1) {
        focusPRN = (uint32_t)atoi(argv[1]);
    }

    try {
        ElasticReceiver rx;
        if (!rx.connect_to_relay("127.0.0.1", 12345)) {
            fclose(out);
            return -1;
        }

        std::cout << "[*] Waiting for stream telemetry..." << std::endl;
        std::vector<uint8_t> startup_buffer(8184);
        if (!rx.get_ms_blocks(startup_buffer.data(), meta, 1)) {
            std::cerr << "[!] No data received from relay." << std::endl;
            fclose(out);
            return -1;
        }

        PCSEngine pcs((float)meta.fs_rate);
        AcquisitionMgr acqMgr(pcs);

        bool acq_needed = true;
        auto start_wall = std::chrono::steady_clock::now();
        double total_data_time = 0;
        bool first = true;

        const int buffer_ms = 5;
        const size_t samples_per_ms = (size_t)(meta.fs_rate / 1000.0);
        const size_t block_size = samples_per_ms * buffer_ms;
        std::vector<uint8_t> block(block_size);
        int epochs_captured = 0;

        while (true) {
            // 1. Pull 5ms of data
            if (rx.get_ms_blocks(block.data(), meta, buffer_ms)) {
                if (first) {
                    start_wall = std::chrono::steady_clock::now();
                    first = false;
                }

                // 2. ACQUISITION PHASE
                if (acq_needed) {
                    printf("[*] Starting Acquisition on 5ms block...\n");
                    auto results = acqMgr.run(meta, block.data());
                    if (!results.empty()) {
                        activeChannels.clear();
                        //activeChannels.reserve(1);

                        for (const auto &res : results) {
                            // Print ALL acquisition results to the terminal window
                            if (res.prn == (int)focusPRN) {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f <--- Exclusive FOCUS\n", res.prn, res.snr, res.bin, res.codePhase);
                                // Store ONLY the focus channel for active tracking
                                activeChannels.emplace_back(res.prn, (double)meta.fs_rate, res, pcs.getSV(res.prn));
                                activeChannels.back().decoder->setFocus(true);
                            } else {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n", res.prn, res.snr, res.bin, res.codePhase);
                            }
                        }

                        if (!activeChannels.empty()) {
                            printf("[*] HANDOVER SUCCESS: %zu focus channel initialized.\n", activeChannels.size());
                            acq_needed = false;
                            rx.jump_to_latest_epoch();
                            continue;
                        } else {
                            printf("[!] Focus PRN %d not found in acquisition pool. Retrying...\n", focusPRN);
                            continue;
                        }
                    } else {
                        printf("[!] No satellites found. Retrying...\n");
                        continue;
                    }
                }

                // 3. TRACKING PHASE (Dynamic Ring Buffer Processing)
                if (!activeChannels.empty()) {
                    auto &state = activeChannels.front();
                    if (state.prn == (int)focusPRN) {
                        
                        // Push the raw incoming 5ms block straight onto our channel's queue
                        state.residualSamples.insert(state.residualSamples.end(), block.begin(), block.end());
                        
                        size_t processedBytesOffset = 0;

                        // Drain the queue in uniform, consecutive 1ms chunk steps
                        while (state.residualSamples.size() - processedBytesOffset >= samples_per_ms) {
                            uint8_t *ms_slice_ptr = state.residualSamples.data() + processedBytesOffset;
                            
                            // Process exactly 1ms worth of samples smoothly
                            CorrelatorResult res = state.processor->Correlator(ms_slice_ptr, samples_per_ms);
                            processedBytesOffset += samples_per_ms;

                            if (res.epoch_valid) {
                                epochs_captured++;

                                // Feed tracking loop outputs cleanly into the NavDecoder state machine
                               // state.decoder->processTrackingMetrics(res);

                                // Captured window lengthened to 12,000ms to allow a full 6-second subframe to pass
                                if (out && epochs_captured <= 12000) {
                                    // Base continuous timeline increments relative to the slice index count
                                    uint64_t slice_sample_tick = meta.sample_tick + processedBytesOffset;
                                    t3 = get_timeData(meta.unix_time, slice_sample_tick, meta.fs_rate);

                                    // Write the timestamp using your safe string conversion method
                                    fprintf(out, "%s ", get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());
                                    
                                    // Use built-in printing tool inside the focus loop
                                    printCorrelatorData(out, res);
                                    fprintf(out, "\n");

                                    // EXTRACT SINGLE COHERENT TRACKING SYMBOL FROM IN-PHASE ENERGY SIGN
                                    fprintf(out, " | Bits: ");
                                    if (std::abs(res.Pi) > 1e+05) {
                                        fprintf(out, "%c", (res.Pi > 0) ? '#' : '-');
                                    } else {
                                        fprintf(out, "?");
                                    }
                                    fprintf(out, "\n");

                                    if (epochs_captured == 12000) {
                                        std::cout << "\n[*] 12 seconds of tracking data captured. Closing file." << std::endl;
                                        fclose(out);
                                        out = nullptr;
                                    }
                                }

                                if (epochs_captured % 100 == 0) {
                                    printCorrelatorData(stdout, res);
                                    fprintf(stdout, "\r");
                                }
                            } // End res.epoch_valid
                        }

                        // Wipe consumed bytes from the queue; let leftover fractions spill into next iteration
                        if (processedBytesOffset > 0) {
                            state.residualSamples.erase(state.residualSamples.begin(), state.residualSamples.begin() + processedBytesOffset);
                        }
                        
                        total_data_time += (double)buffer_ms / 1000.0;
                    }
                }
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        } // end while true
    }
    catch (const std::exception &e) {
        std::cerr << "\n[!] Exception: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "\n[!] Unknown Error in SDR_test loop." << std::endl;
    }

    if (out) fclose(out);
    return 0;
}
