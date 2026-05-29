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
    if (argc > 1 && argv[1] != nullptr)
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
        // ========================================================================
        // REFACTORED MASTER PROCESSING LOOP (STRICT STRIDE SYNCHRONIZATION)
        // ========================================================================
        bool first = true;
        const int buffer_ms = 5;
        const size_t samples_per_ms = (size_t)(meta.fs_rate / 1000.0);
        const size_t block_size = samples_per_ms * buffer_ms;
        std::vector<uint8_t> block(block_size);
        int epochs_captured = 0;

        std::cout << "[*] Starting real-time hardware tracking loop..." << std::endl;

        while (true)
        {
            // 1. NETWORK INGEST: Pull exactly ONE distinct 5ms block per cycle to guarantee alignment
            if (rx.get_ms_blocks(block.data(), meta, buffer_ms))
            {
                if (first)
                {
                    start_wall = std::chrono::steady_clock::now();
                    first = false;
                }
                stuffFIFO(rxState.sampleFIFO, block.data(), block_size, meta.sample_tick, meta.unix_time);
                rxState.totalSamplesPushed += block_size * 2;
            }
            else
            {
                // No new data waiting on socket: drop thread pressure and retry ingest
                std::this_thread::sleep_for(std::chrono::microseconds(250));
                continue;
            }

            // 2. ACQUISITION HANDOVER: Guarded handover baseline mapping
            if (acq_needed)
            {
                if (rxState.sampleFIFO.size() >= block_size * 2)
                {
                    auto results = acqMgr.run(meta, block.data());
                    t3 = get_timeData(meta.unix_time, meta.sample_tick, meta.fs_rate);

                    if (!results.empty())
                    {
                        activeChannels.clear();
                        const AcqResult *focusTarget = nullptr;

                        for (const auto &res : results)
                        {
                            if (res.prn == (int)focusPRN)
                            {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f <--- Focus\n", res.prn, res.snr, res.bin, res.codePhase);
                                focusTarget = &res;
                            }
                            else
                            {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n", res.prn, res.snr, res.bin, res.codePhase);
                            }
                        }

                        if (focusTarget != nullptr)
                        {
                            activeChannels.emplace_back(focusTarget->prn, (double)meta.fs_rate, *focusTarget, pcs.getSV(focusTarget->prn));
                            activeChannels.back().decoder->setFocus(true);

                            auto &state = activeChannels.front();
                            state.totalSamplesPushed = rxState.totalSamplesPushed;
                            state.totalSamplesConsumed = 0;
                            state.handover_sample_tick = (uint32_t)std::round(focusTarget->codePhase);
                            state.handover_unix_time = meta.unix_time;

                            acq_needed = false;
                            printf("[*] HANDOVER SUCCESS: Channel anchored to stream timeline. No skips.\n");
                            continue;
                        }
                        else
                        {
                            rxState.resetPipelines();
                            continue;
                        }
                    }
                }
            }

            // ========================================================================
            // 3. MULTI-CHANNEL PROCESSING: Direct 1ms Step Evaluation
            // ========================================================================
            if (!activeChannels.empty())
            {
                const size_t HARDWARE_MS_BLOCK = 16368;
                const size_t TOTAL_BLOCK_SAMPLES = buffer_ms * HARDWARE_MS_BLOCK; // 81840 samples

                // Process chunks continuously as they are drained from the incoming queue
                while (rxState.sampleFIFO.size() >= TOTAL_BLOCK_SAMPLES)
                {

                    bool chunkProcessed = true;

                    for (auto &state : activeChannels)
                    {
                        if (state.prn == (int)focusPRN)
                        {

                            // Feed the correlator 1 ms at a time across your 5 ms network block
                            for (int ms_step = 0; ms_step < buffer_ms; ++ms_step)
                            {

                                // Slice out a clean 1ms snapshot starting exactly from the current step offset
                                auto startIt = rxState.sampleFIFO.begin() + (ms_step * HARDWARE_MS_BLOCK);
                                std::vector<RawSample> flatSlice(startIt, startIt + HARDWARE_MS_BLOCK);

                                // Feed exactly 1ms of data to the correlator
                                CorrelatorResult res = state.processor->Correlator(flatSlice.data(), flatSlice.size());
                                const RawSample &lastSample = flatSlice.back();
                                // Codephase tracking relative to the sample clock layout
                                double activeCodeFreq = 1023000.0 + (double)res.doppler_hz / 1540.0;

                                if (res.rollover_sample_index_in_block != -1)
                                {
                                    double dynamic_samples_per_chip = meta.fs_rate / activeCodeFreq;
                                    double chips_to_end = (double)res.rollover_sample_index_in_block / dynamic_samples_per_chip;
                                    state.result.codePhase = std::fmod(1023.0 - chips_to_end, 1023.0);
                                }
                                else
                                {
                                    double chips_advanced = (HARDWARE_MS_BLOCK * activeCodeFreq) / meta.fs_rate;
                                    state.result.codePhase = std::fmod(state.result.codePhase + chips_advanced, 1023.0);
                                }

                                res.code_phase = state.result.codePhase;
                                state.handover_sample_tick = (uint32_t)std::round(res.code_phase);

                                // Telemetry logging driven entirely by the physical sample stamps
                                // Telemetry logging driven entirely by the physical sample stamps
                                if (res.epoch_valid)
                                {
                                    epochs_captured++;

                                    // FIX: Recombine whole seconds and resetting clock ticks directly into a unified
                                    // millisecond timeline baseline to prevent any visual stutters or backward steps.
                                    uint64_t sample_unix_time = lastSample.unix_time;
                                    uint64_t current_tick = lastSample.sample_tick;
                                    double samples_per_second = meta.fs_rate;

                                    // 1. Isolate the precise fractional millisecond component (0-999) from the resetting clock
                                    uint64_t fractional_ms = (uint64_t)((current_tick % (uint64_t)samples_per_second) * 1000 / samples_per_second);

                                    // 2. RECONCILE TIMELINE: Build a continuous millisecond-epoch value using your sample stream fields.
                                    // If the current hardware tick count is very low (indicating a reset to 0 has occurred),
                                    // but the incoming packet-header unix_time hasn't ticked forward yet, we check the loop step
                                    // to force the whole seconds column forward seamlessly.
                                    uint64_t total_ms_since_epoch = (sample_unix_time * 1000) + fractional_ms;

                                    // If we have processed multiple steps, but the fractional ms abruptly wraps around near zero,
                                    // the physical hardware second rolled over mid-block. We force the seconds column forward.
                                    if (ms_step > 0 && fractional_ms < 10)
                                    {
                                        // If sample_unix_time hasn't rolled forward yet, add 1000ms to advance the second column
                                        total_ms_since_epoch += 1000;
                                    }

                                    t3.unixSecond = total_ms_since_epoch / 1000;
                                    t3.msCount = (uint32_t)(total_ms_since_epoch % 1000);

                                    fprintf(out, "%s ", get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());
                                    printCorrelatorData(out, res);
                                    fprintf(out, " | Bits: %c\n", (res.Pi > 0) ? '#' : '-');

                                    if (epochs_captured % 100 == 0)
                                    {
                                        printCorrelatorData(stdout, res);
                                        fprintf(stdout, "\n");
                                        fflush(stdout);
                                    }
                                }
                            } // End 1ms step loop
                        }
                    } // End channel loop

                    // ========================================================================
                    // 4. STREAM CLEANUP: Evict the evaluated block completely from the front
                    // ========================================================================
                    rxState.sampleFIFO.erase(rxState.sampleFIFO.begin(), rxState.sampleFIFO.begin() + TOTAL_BLOCK_SAMPLES);

                    for (auto &state : activeChannels)
                    {
                        state.totalSamplesConsumed = 0;
                    }
                } // End drainage while loop

                total_data_time += (double)buffer_ms / 1000.0;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }

        } // End while(true)
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
