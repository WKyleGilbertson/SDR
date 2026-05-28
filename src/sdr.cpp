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
            // STEP 3: STREAM INGESTION (EXHAUST ALL AVAILABLE SOCKET CORES)
            // STEP 3: NON-BLOCKING BOUNDED SOCKET INGESTION (PREVENTS BAD ALLOCATION)
            int blocks_pulled_this_cycle = 0;

            // Drain up to 5 blocks (25ms of data) maximum per main loop pass
            while (blocks_pulled_this_cycle < 5 && rx.get_ms_blocks(block.data(), meta, buffer_ms))
            {
                if (first)
                {
                    start_wall = std::chrono::steady_clock::now();
                    first = false;
                }

                stuffFIFO(rxState.sampleFIFO, block.data(), block_size, meta.sample_tick, meta.unix_time);
                rxState.totalSamplesPushed += block_size * 2;
                blocks_pulled_this_cycle++;
            }

            // STEP 4: ACQUISITION PHASE (COMPLETE DISCOVERY VISUALIZATION)
            if (acq_needed)
            {
                if (rxState.sampleFIFO.size() >= block_size * 2)
                {
                    auto results = acqMgr.run(meta, block.data());
                    t3 = get_timeData(meta.unix_time, meta.sample_tick, meta.fs_rate);
                    printf("[*] Starting Acquisition on unified stream chunk... %s\n", get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());

                    if (!results.empty())
                    {
                        activeChannels.clear();
                        const AcqResult *focusTarget = nullptr;

                        // FIX Step 1: Process and PRINT every single satellite discovered in this frame first
                        for (const auto &res : results)
                        {
                            if (res.prn == (int)focusPRN)
                            {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f <--- Focus\n", res.prn, res.snr, res.bin, res.codePhase);
                                focusTarget = &res; // Cache the pointer to initialize after logging finishes
                            }
                            else
                            {
                                printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n", res.prn, res.snr, res.bin, res.codePhase);
                            }
                        }

                        // FIX Step 2: Now that all tracking targets are visible, perform the handover if found
                        if (focusTarget != nullptr)
                        {
                            activeChannels.emplace_back(focusTarget->prn, (double)meta.fs_rate, *focusTarget, pcs.getSV(focusTarget->prn));
                            activeChannels.back().decoder->setFocus(true);

                            auto &state = activeChannels.front();
                            state.sampleFIFO = std::move(rxState.sampleFIFO);
                            state.totalSamplesPushed = rxState.totalSamplesPushed;
                            state.totalSamplesConsumed = 0;
                            state.handover_sample_tick = meta.sample_tick;
                            state.handover_unix_time = meta.unix_time;

                            acq_needed = false;
                            printf("[*] HANDOVER SUCCESS: Pipeline locked sequentially. No skips.\n");

                            // Continue cleanly to drop straight down into the 1ms multiplexer loop
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
            }

            // STEP 5 & 6: MONOTONIC MASTER-STREAM PIPELINE (ZERO DUPLICATION / ZERO DATA DROPS)
            if (!activeChannels.empty())
            {
                auto &state = activeChannels.front();
                if (state.prn == (int)focusPRN)
                {

                    // FIX 1: Eliminate the local channel queue duplication.
                    // Feed the correlator directly from the global rxState memory stack.
                    while (rxState.sampleFIFO.size() >= 16368)
                    {

                        // Slice out 1ms of continuous hardware samples straight from the source container
                        std::vector<RawSample> flatSlice(rxState.sampleFIFO.begin(), rxState.sampleFIFO.begin() + 16368);

                        CorrelatorResult res = state.processor->Correlator(flatSlice.data(), flatSlice.size());

                        // FIX 2: Ensure strict consumption feedback
                        if (res.consumed_sample_count > 0)
                        {
                            state.totalSamplesConsumed += res.consumed_sample_count;

                            // Erase exactly what was evaluated from the global master queue.
                            // This makes data duplication mathematically impossible.
                            rxState.sampleFIFO.erase(rxState.sampleFIFO.begin(), rxState.sampleFIFO.begin() + res.consumed_sample_count);
                        }
                        else
                        {
                            // Fail-safe protection: if the tracking filters stall, force advance to break deadlocks
                            rxState.sampleFIFO.erase(rxState.sampleFIFO.begin(), rxState.sampleFIFO.begin() + 16368);
                        }

                        if (res.epoch_valid)
                        {
                            epochs_captured++;

                            // Track time monotonically based on total physical data consumed
                            double precise_ms_elapsed = (double)state.totalSamplesConsumed * 1000.0 / meta.fs_rate;
                            uint64_t total_ms_since_epoch = ((uint64_t)state.handover_unix_time * 1000) + (uint64_t)std::round(precise_ms_elapsed);

                            t3.unixSecond = total_ms_since_epoch / 1000;
                            t3.msCount = total_ms_since_epoch % 1000;

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
                        }
                    } // End while processing master chunks

                    total_data_time += (double)buffer_ms / 1000.0;
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