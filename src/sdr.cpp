#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <thread>
#include <numeric>
#include <cstdio>
#include <cstdint>

#include "L1IFUtil.hpp"
#include "versionInfo.hpp"
#include "ElasticReceiver.h"
#include "AcquisitionMgr.hpp"
#include "ChannelProcessor.h"
#include "PCSEngine.hpp"
#include "NavDecoder.h"

struct ChannelState
{
    int prn;
    AcqResult result;                            // Metadata from PCS (PRN, CodePhase, Bin)
    G2INIT sv;                                   // The Gold Code replica (bits)
    std::unique_ptr<ChannelProcessor> processor; // The active tracker
    std::unique_ptr<NavDecoder> decoder;

    bool isActive() const { return processor != nullptr; }

    ChannelState(int p, double fs, const AcqResult &res, G2INIT s) : prn(p), result(res), sv(s)
    {
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

int main(int argc, char *argv[])
{
    FILE *out = fopen("output.bin", "wb");
    if (!out)
    {
        std::cerr << "[!] Failed to open output file." << std::endl;
        return -1;
    }

    versionInfo v;
    v.printVersion();
    RFE_Header_t meta = {};
    TimeTrio t3;
    std::vector<ChannelState> activeChannels;
    auto system_start = std::chrono::steady_clock::now();
    uint32_t focusPRN = 131;
    if (argc > 1)
    {
        focusPRN = (uint32_t)atoi(argv[1]);
    }

    try
    {
        ElasticReceiver rx;
        if (!rx.connect_to_relay("127.0.0.1", 12345))
        {
            fclose(out);
            return -1;
        }
        std::cout << "[*] Waiting for stream telemetry..." << std::endl;
        std::vector<uint8_t> startup_buffer(8184);
        if (!rx.get_ms_blocks(startup_buffer.data(), meta, 1))
        {
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

        while (true)
        {
            // 1. Pull 5ms of data
            if (rx.get_ms_blocks(block.data(), meta, buffer_ms))
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
                        activeChannels.reserve(1);
                        for (const auto &res : results)
                        {
                            // Print ALL acquisition results to the terminal window
                            if (res.prn == (int)focusPRN)
                            {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f <--- Exclusive FOCUS\n", res.prn, res.snr, res.bin, res.codePhase);
                                // Store ONLY the focus channel for active tracking
                                activeChannels.emplace_back(res.prn, (double)meta.fs_rate, res, pcs.getSV(res.prn));
                                activeChannels.back().decoder->setFocus(true);
                            }
                            else
                            {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n", res.prn, res.snr, res.bin, res.codePhase);
                            }
                        }

                        if (!activeChannels.empty())
                        {
                            printf("[*] HANDOVER SUCCESS: %zu focus channel initialized.\n", activeChannels.size());
                            acq_needed = false;
                            rx.jump_to_latest_epoch();
                            continue;
                        }
                        else
                        {
                            printf("[!] Focus PRN %d not found in acquisition pool. Retrying...\n", focusPRN);
                            continue;
                        }
                    }
                    else
                    {
                        printf("[!] No satellites found. Retrying...\n");
                        continue;
                    }
                }
                // 3. TRACKING PHASE (Steps through the 5ms buffer in five sequential 1ms iterations)
                if (!activeChannels.empty())
                {
                    auto &state = activeChannels.front();

                    if (state.prn == (int)focusPRN)
                    {
                        for (int ms_step = 0; ms_step < buffer_ms; ms_step++)
                        {
                            // Address pointer arithmetic to slice the 5ms block into sequential 1ms windows
                            uint8_t *ms_slice_ptr = block.data() + (ms_step * samples_per_ms);

                            // Feed exactly 1 ms of sample data to the tracking loop
                            CorrelatorResult res = state.processor->Correlator(ms_slice_ptr, samples_per_ms);

                            if (res.epoch_valid) {
                            // Maintain continuous streaming time increments for the tracking NCO
                            uint64_t slice_sample_tick = meta.sample_tick + (ms_step * samples_per_ms);
                            t3 = get_timeData(meta.unix_time, slice_sample_tick, meta.fs_rate);

                            // Write the timestamp using your safe string conversion method
                            fprintf(out, "%s ", get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());

                            // Use built-in printing tool inside the focus loop
                            printCorrelatorData(out, res);
                            fprintf(out, "\n");

                            // EXTRACT SINGLE COHERENT TRACKING SYMBOL FROM IN-PHASE ENERGY SIGN
                            fprintf(out, " | Bits: ");

                            // If the prompt energy is robust, use its raw sign as the true data bit symbol
                            if (std::abs(res.Pi) > 1e+05)
                            {
                                fprintf(out, "%c", (res.Pi > 0) ? '#' : '-');
                            }
                            else
                            {
                                // If power is lost or drifting due to a cycle slip, flag it clearly
                                fprintf(out, "?");
                            }
                            fprintf(out, "\n");
                        }
                        }
                        total_data_time += (double)buffer_ms / 1000.0;
                    }
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

    fclose(out);
    return 0;
}
