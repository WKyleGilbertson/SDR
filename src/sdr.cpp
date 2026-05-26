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

    uint64_t handover_sample_tick = 0;
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

    // Master receiver initialization state tracking layer
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

                // FIXED: Cleaned and flattened unscaled conversion sweep loop metrics
                const UnpackEntry *lut = GetLUT_FNHN();
                uint64_t tick0 = 0;
                uint64_t tick1 = 0;

                for (size_t i = 0; i < block_size; ++i)
                {
                    const UnpackEntry &entry = lut[block[i]];

                    tick0 = meta.sample_tick + (i * 2);
                    tick1 = meta.sample_tick + (i * 2) + 1;

                    RawSample s0 = {static_cast<int8_t>(entry.s0.i), static_cast<int8_t>(entry.s0.q), tick0, meta.unix_time};
                    rxState.sampleFIFO.push_back(s0);
                    rxState.totalSamplesPushed++;

                    RawSample s1 = {static_cast<int8_t>(entry.s1.i), static_cast<int8_t>(entry.s1.q), tick1, meta.unix_time};
                    rxState.sampleFIFO.push_back(s1);
                    rxState.totalSamplesPushed++;
                }

                // STEP 4: ACQUISITION PHASE
                if (acq_needed)
                {
                    auto results = acqMgr.run(meta, block.data());
                    t3 = get_timeData(meta.unix_time, meta.sample_tick, meta.fs_rate);
                    printf("[*] Starting Acquisition on unified stream chunk... %s\n",
                           get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());
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

                                // Seeding baseline pipeline variables across handover
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
                            rx.jump_to_latest_epoch();
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

                // STEP 5 & 6: PIPELINE TRACKING MULTIPLEXER LAYER
                if (!activeChannels.empty())
                {
                    auto &state = activeChannels.front();
                    if (state.prn == (int)focusPRN)
                    {

                        if (&state != &rxState && !rxState.sampleFIFO.empty())
                        {
                            state.sampleFIFO.insert(state.sampleFIFO.end(), rxState.sampleFIFO.begin(), rxState.sampleFIFO.end());
                            state.totalSamplesPushed += rxState.totalSamplesPushed;
                            rxState.resetPipelines();
                        }

                        bool epochsAvailable = true;
                        while (epochsAvailable && !state.sampleFIFO.empty())
                        {
                            std::vector<RawSample> flatSlice(state.sampleFIFO.begin(), state.sampleFIFO.end());
                            CorrelatorResult res = state.processor->Correlator(flatSlice.data(), flatSlice.size());

                            if (res.epoch_valid && res.consumed_sample_count > 0)
                            {
                                epochs_captured++;
                                state.totalSamplesConsumed += res.consumed_sample_count;

                                // Discard precisely what was tracked out of the FIFO queue
                                state.sampleFIFO.erase(state.sampleFIFO.begin(), state.sampleFIFO.begin() + res.consumed_sample_count);
                             //   state.decoder->processTrackingMetrics(res);

                                if (out && epochs_captured <= 12000)
                                {

                                    // 1. Calculate the current epoch's millisecond offset inside the block
                                    // based on the unshifted hardware block snapshot timing data
                                    uint32_t currentBlockMsOffset = (state.handover_sample_tick / 16368) % 1000;

                                    // 2. Compute the smooth, sequential millisecond tick count
                                    uint64_t dynamicMsCounter = currentBlockMsOffset + state.total_tracked_ms;

                                    // 3. Construct a precise, sample-accurate hardware tracking tick
                                    uint64_t preciseSampleTick = (uint64_t)(dynamicMsCounter * 16368);

                                    // Extract the calendar date and millisecond tracking step increments cleanly
                                    t3 = get_timeData(state.handover_unix_time, preciseSampleTick, 16368000.0);

                                    fprintf(out, "%s ", get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());
                                    printCorrelatorData(out, res);
                                    fprintf(out, " | Bits: %c\n", (res.Pi > 0) ? '#' : '-');

                                    // 4. CRITICAL STEP: Increment the persistent tracking timeline
                                    state.total_tracked_ms++;

                                    if (epochs_captured == 12000)
                                    {
                                        std::cout << "\n[*] 12 seconds captured. Closing file." << std::endl;
                                        fclose(out);
                                        out = nullptr;
                                    }
                                }

                                if (epochs_captured % 100 == 0)
                                {
                                    printCorrelatorData(stdout, res);
                                    fprintf(stdout, "\n");
                                    fflush(stdout);
                                }
                            }
                            else
                            {
                                epochsAvailable = false;
                            }
                        } // End while epochsAvailable

                        total_data_time += (double)buffer_ms / 1000.0;
                    }
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        } // end while true
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
