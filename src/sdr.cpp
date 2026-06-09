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
    uint64_t sampleCursor = 0; // Absolute sample index for tracking where we are in the stream
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
    static int dbg_counter = 0;
    const uint64_t max_logged_ms = 250;
    uint64_t logged_ms = 0;
    bool file_logging_enabled = true;
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
        const int tracking_ms = 1;
        const int acq_ms = 5;
        const size_t ms_samples = (size_t)(meta.fs_rate / 1000.0);
        const size_t acq_samples = ms_samples * acq_ms;

        std::cout << "[*] Starting real-time hardware tracking loop..." << std::endl;

        while (true)
        {
            dbg_counter++;

            if (dbg_counter % 200 == 0)
            {
                if (rx.validate_ring_continuity())
                {
                    printf("[RING OK] write=%llu\n", rx.get_write_index());
                }
            }

            if (acq_needed)
            {
                uint64_t newest = rx.get_write_index();

                if (newest < acq_samples + ms_samples)
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(250));
                    continue;
                }

                // Start with latest complete 5 ms region.
                uint64_t acq_cursor = newest - acq_samples;

                RawSample *acq_ptr = nullptr;
                std::vector<RawSample> acq_window;

                if (!rx.get_window(acq_cursor, acq_ptr, (unsigned int)acq_samples, acq_window))
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(250));
                    continue;
                }

                // Align by hardware sample_tick, not ring index.
                uint32_t tick_mod = acq_ptr[0].sample_tick % (uint32_t)ms_samples;

                if (tick_mod != 0)
                {
                    if (acq_cursor < tick_mod)
                    {
                        std::this_thread::sleep_for(std::chrono::microseconds(250));
                        continue;
                    }

                    acq_cursor -= tick_mod;

                    if (!rx.get_window(acq_cursor, acq_ptr, (unsigned int)acq_samples, acq_window))
                    {
                        std::this_thread::sleep_for(std::chrono::microseconds(250));
                        continue;
                    }
                }

                printf(
                    "[ACQ] tick=%u mod=%u samples=%zu cursor=%llu\n",
                    acq_ptr[0].sample_tick,
                    acq_ptr[0].sample_tick % (uint32_t)ms_samples,
                    acq_samples,
                    acq_cursor);

                auto results = acqMgr.run(meta, acq_ptr, acq_samples);

                if (!results.empty())
                {
                    activeChannels.clear();

                    const AcqResult *focusTarget = nullptr;

                    for (const auto &res : results)
                    {
                        if (res.prn == (int)focusPRN)
                        {
                            printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f <--- Focus\n",
                                   res.prn, res.snr, res.bin, res.codePhase);
                            focusTarget = &res;
                        }
                        else
                        {
                            printf(" LOCKED | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n",
                                   res.prn, res.snr, res.bin, res.codePhase);
                        }
                    }

                    if (focusTarget != nullptr)
                    {
                        activeChannels.emplace_back(
                            focusTarget->prn,
                            (double)meta.fs_rate,
                            *focusTarget,
                            pcs.getSV(focusTarget->prn));

                        activeChannels.back().decoder->setFocus(true);

                        auto &state = activeChannels.front();

                        state.totalSamplesConsumed = 0;
                        state.total_tracked_ms = 0;
                        state.handover_sample_tick = (uint32_t)std::round(focusTarget->codePhase);
                        state.handover_unix_time = acq_ptr[0].unix_time;

                        // Important: add this field to ChannelState next.
                        //                        state.sampleCursor = acq_cursor + acq_samples;
                        uint64_t write = rx.get_write_index();
                        uint64_t aligned_write = write - (write % ms_samples);

                        state.sampleCursor = aligned_write - ms_samples;

                        acq_needed = false;

                        printf("[*] HANDOVER SUCCESS: acquisition window [%llu, %llu)\n",
                               acq_cursor,
                               acq_cursor + acq_samples);

                        continue;
                    }
                }

                continue;
            }
            /* Tracking goes here */
            if (!activeChannels.empty())
            {
                for (auto &state : activeChannels)
                {
                    if (state.prn != (int)focusPRN)
                        continue;

                    uint64_t write = rx.get_write_index();
                    uint64_t ring_capacity = ms_samples * 250;

                    if (state.sampleCursor + ms_samples < write - ring_capacity)
                    {
                        printf(
                            "[TRK STALE] PRN %d cursor=%llu write=%llu capacity=%llu -- reacquire\n",
                            state.prn,
                            state.sampleCursor,
                            write,
                            ring_capacity);

                        acq_needed = true;
                        activeChannels.clear();
                        break;
                    }

                    while (rx.get_write_index() >= state.sampleCursor + ms_samples)
                    {
                        RawSample *ms_ptr = nullptr;
                        std::vector<RawSample> ms_window;

                        if (!rx.get_window(
                                state.sampleCursor,
                                ms_ptr,
                                (unsigned int)ms_samples,
                                ms_window))
                        {
                            break;
                        }

                        CorrelatorResult res =
                            state.processor->Correlator(
                                ms_ptr,
                                (unsigned int)ms_samples);

                        state.sampleCursor += ms_samples;
                        state.total_tracked_ms++;

                        // ---- Existing code phase update ----

                        double activeCodeFreq =
                            1023000.0 +
                            (double)res.doppler_hz / 1540.0;

                        if (res.rollover_sample_index_in_block != -1)
                        {
                            double dynamic_samples_per_chip =
                                (double)(ms_samples * 1000.0) /
                                activeCodeFreq;

                            double chips_to_end =
                                (double)res.rollover_sample_index_in_block /
                                dynamic_samples_per_chip;

                            state.result.codePhase =
                                std::fmod(
                                    1023.0 - chips_to_end,
                                    1023.0);
                        }
                        else
                        {
                            double chips_advanced =
                                (double)ms_samples *
                                activeCodeFreq /
                                (double)(meta.fs_rate);

                            state.result.codePhase =
                                std::fmod(
                                    state.result.codePhase +
                                        chips_advanced,
                                    1023.0);
                        }

                        res.code_phase =
                            state.result.codePhase;

                        // ---- Timing comes directly from ring ----

                        if (res.epoch_valid)
{
    const RawSample& sample = ms_ptr[0];

    uint64_t stable_fs_rate = (uint64_t)meta.fs_rate;
    uint64_t sub_second_ticks = sample.sample_tick % stable_fs_rate;
    uint32_t calculated_ms =
        (uint32_t)((sub_second_ticks * 1000) / stable_fs_rate);

    t3.unixSecond = sample.unix_time;
    t3.msCount = calculated_ms;

    // 1 symbol per 1 ms integration
    const char symbol = (res.Pi > 0) ? '#' : '-';

    if (file_logging_enabled && logged_ms < max_logged_ms)
    {
        fprintf(
            out,
            "%s ",
            get_iso8601_timestamp(t3.unixSecond, t3.msCount).c_str());

        printCorrelatorData(out, res);

        fprintf(out, " | Bits: %c\n", symbol);

        logged_ms++;

        if (logged_ms == max_logged_ms)
        {
            fflush(out);
            file_logging_enabled = false;
            printf("[LOG] Stopped file logging after %llu ms\n", logged_ms);
        }
    }

    // Lightweight console sanity check
    if (state.total_tracked_ms % 100 == 0)
    {
        printf(
            "[TRK] PRN %3d SNR:%5.1f dF:%8.1f Code:%7.2f Pi:% .0f Pq:% .0f Bit:%c\n",
            state.prn,
            res.snr,
            res.doppler_hz,
            res.code_phase,
            res.Pi,
            res.Pq,
            symbol);
    }
}

                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(250));
        } // End of while(true)
    } // End try
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
