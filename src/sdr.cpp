#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <deque>
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

struct ChannelState
{
    int prn;
    AcqResult result;
    G2INIT sv;
    std::unique_ptr<ChannelProcessor> processor;
    std::unique_ptr<NavDecoder> decoder;
    std::deque<RawSample> sampleFIFO;
    uint64_t totalSamplesPushed = 0;
    uint64_t totalSamplesConsumed = 0;
    uint64_t total_tracked_ms = 0;
    uint32_t handover_sample_tick = 0;
    uint32_t handover_unix_time = 0;

    bool isActive() const { return processor != nullptr; }

    ChannelState(int p, double fs, const AcqResult &res, G2INIT s) : prn(p), result(res), sv(s)
    {
        processor = std::make_unique<ChannelProcessor>(fs, result, s);
        decoder = std::make_unique<NavDecoder>(p);
    }

    void resetPipelines()
    {
        sampleFIFO.clear();
        totalSamplesPushed = 0;
        totalSamplesConsumed = 0;
    }
};

void stuffFIFO(std::deque<RawSample> &fifo, const uint8_t *data, size_t count, uint32_t base_sample_tick, uint32_t unix_time)
{
    const UnpackEntry *lut = GetLUT_FNHN();
    for (size_t i = 0; i < count; ++i)
    {
        const UnpackEntry &entry = lut[data[i]];
        RawSample s0;
        s0.i = static_cast<int8_t>(entry.s0.i);
        s0.q = static_cast<int8_t>(entry.s0.q);
        s0.sample_tick = base_sample_tick + (i * 2);
        s0.unix_time = unix_time;
        fifo.push_back(s0);

        RawSample s1;
        s1.i = static_cast<int8_t>(entry.s1.i);
        s1.q = static_cast<int8_t>(entry.s1.q);
        s1.sample_tick = base_sample_tick + (i * 2) + 1;
        s1.unix_time = unix_time;
        fifo.push_back(s1);
    }
}

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
    std::list<ChannelState> activeChannels;

    ChannelState rxState(0, 16368000.0, AcqResult(), G2INIT());
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
        int epochs_captured = 0;

        while (true)
        {
            // STEP 3: Stream Ingestion
            if (rx.get_ms_blocks(block.data(), meta, buffer_ms))
            {
                if (first)
                {
                    start_wall = std::chrono::steady_clock::now();
                    first = false;
                }

                stuffFIFO(rxState.sampleFIFO, block.data(), block_size, meta.sample_tick, meta.unix_time);
                rxState.totalSamplesPushed += block_size * 2;

                // STEP 4: ACQUISITION PHASE
                if (acq_needed)
                {
                    auto results = acqMgr.run(meta, block.data());
                    t3 = get_timeData(meta.unix_time, meta.sample_tick, meta.fs_rate);
                    printf("[*] Starting Acquisition on unified stream chunk... %s\n", get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());

                    if (!results.empty())
                    {
                        activeChannels.clear();
                        for (const auto &res : results)
                        {
                            if (res.prn == (int)focusPRN)
                            {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f <--- Focus\n", res.prn, res.snr, res.bin, res.codePhase);
                                activeChannels.emplace_back(res.prn, (double)meta.fs_rate, res, pcs.getSV(res.prn));
                                activeChannels.back().decoder->setFocus(true);

                                auto &state = activeChannels.front();
                                state.sampleFIFO = std::move(rxState.sampleFIFO);
                                state.totalSamplesPushed = rxState.totalSamplesPushed;
                                state.totalSamplesConsumed = 0;
                                state.handover_sample_tick = meta.sample_tick;
                                state.handover_unix_time = meta.unix_time;
                                state.total_tracked_ms = 0;
                                acq_needed = false;
                            }
                            else
                            {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n", res.prn, res.snr, res.bin, res.codePhase);
                            }
                        }

                        if (!acq_needed)
                        {
                            printf("[*] HANDOVER SUCCESS: Unscaled pipeline tracking active.\n");
                            rxState.resetPipelines(); // Maybe not?
                            rx.jump_to_latest_epoch();
                            // CRITICAL FIX: Update the base tracking timeline parameters immediately
                            // to match the stream state after skipping network frames.
                            if (!activeChannels.empty())
                            {
                                auto &state = activeChannels.front();
                                state.handover_sample_tick = meta.sample_tick;
                                state.handover_unix_time = meta.unix_time;
                            }
                            continue;
                        }
                        else
                        {
                            printf("[!] Focus PRN %d not found in pool. Retrying...\n", focusPRN);
                            rxState.resetPipelines();
                            continue;
                        }
                    }
                    else
                    {
                        printf("[!] No satellites found. Retrying...\n");
                        rxState.resetPipelines();
                        continue;
                    }
                }

                // STEP 5 & 6: SMOOTHED 1MS DISCRETE STREAM MULTIPLEXER (NO DUPLICATION)
                if (!activeChannels.empty())
                {
                    auto &state = activeChannels.front();
                    if (state.prn == (int)focusPRN)
                    {

                        // Append fresh 5ms chunks to the smoothing FIFO buffer
                        if (&state != &rxState && !rxState.sampleFIFO.empty())
                        {
                            state.sampleFIFO.insert(state.sampleFIFO.end(), rxState.sampleFIFO.begin(), rxState.sampleFIFO.end());
                            state.totalSamplesPushed += rxState.totalSamplesPushed;
                            rxState.resetPipelines();
                        }

                        // STEP 5 & 6: SMOOTHED PASS-THROUGH TELEMETRY MULTIPLEXER
                        while (state.sampleFIFO.size() >= 16368)
                        {
                            std::vector<RawSample> flatSlice(state.sampleFIFO.begin(), state.sampleFIFO.begin() + 16368);

                            CorrelatorResult res = state.processor->Correlator(flatSlice.data(), flatSlice.size());

                            if (res.consumed_sample_count > 0)
                            {
                                state.totalSamplesConsumed += res.consumed_sample_count;
                                state.sampleFIFO.erase(state.sampleFIFO.begin(), state.sampleFIFO.begin() + res.consumed_sample_count);
                            }
                            else
                            {
                                state.sampleFIFO.erase(state.sampleFIFO.begin(), state.sampleFIFO.begin() + 16368);
                            }

                            // CRITICAL FIX: Always update the core clock state on every single block
                            // to maintain synchronicity with incoming network telemetry frames
                            t3 = get_timeData(res.unix_time, res.rollover_sample_idx, meta.fs_rate);

                            // LOG THROTTLE: Only print a row when the correlator crosses a true code boundary
                            if (res.epoch_valid) {
    epochs_captured++;
    
    // 1. Calculate the absolute continuous milliseconds that have elapsed since handover
    uint64_t total_samples_tracked = state.totalSamplesConsumed;
    uint64_t continuous_ms_elapsed = (uint64_t)std::round((double)total_samples_tracked * 1000.0 / meta.fs_rate);
    
    // 2. Compute a continuous time baseline from the initial handover values
    uint64_t absolute_starting_ms = ((uint64_t)state.handover_unix_time * 1000);
    uint64_t current_absolute_ms  = absolute_starting_ms + continuous_ms_elapsed;
    
    // 3. Separate the components cleanly to prevent any backward timeline shifts
    t3.unixSecond = current_absolute_ms / 1000;
    t3.msCount    = current_absolute_ms % 1000;
    
    // Log the synchronized tracking metrics cleanly
    fprintf(out, "%s ", get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());
    printCorrelatorData(out, res);
    fprintf(out, " | Bits: %c\n", (res.Pi > 0) ? '#' : '-');
    
    state.total_tracked_ms++;

    if (epochs_captured % 100 == 0) {
        printCorrelatorData(stdout, res);
        fprintf(stdout, "\r");
        fflush(stdout);
    }
}

                            /*
                            if (res.epoch_valid)
                            {
                                epochs_captured++;

                                fprintf(out, "%s ", get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());
                                printCorrelatorData(out, res);
                                fprintf(out, " | Bits: %c\n", (res.Pi > 0) ? '#' : '-');

                                state.total_tracked_ms++;

                                if (epochs_captured % 100 == 0)
                                {
                                    printCorrelatorData(stdout, res);
                                    fprintf(stdout, "\r");
                                    fflush(stdout);
                                }
                            } */
                        } // End while

                        total_data_time += (double)buffer_ms / 1000.0;
                    }
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        } // End while true
    }
    catch (const std::exception &e)
    {
        std::cerr << "\n[!] Exception: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "\n[!] Unknown Error in SDR_test loop." << std::endl;
    }

    if (out)
        fclose(out);
    return 0;
}