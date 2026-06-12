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
#include "TrackingEngine.h"
#include "NavDecoder.h"

static bool runFreshFocusedAcquisition(
    ElasticReceiver &rx,
    AcquisitionMgr &acqMgr,
    const RFE_Header_t &meta,
    uint32_t focusPRN,
    size_t ms_samples,
    size_t acq_samples,
    AcqResult &fresh,
    uint64_t &fresh_cursor)
{
    uint64_t write = rx.get_write_index();

    uint64_t aligned_write =
        write - (write % ms_samples);

    if (aligned_write < acq_samples)
        return false;

    fresh_cursor =
        aligned_write - acq_samples;

    RawSample *fresh_ptr = nullptr;
    std::vector<RawSample> fresh_window;

    if (!rx.get_window(
            fresh_cursor,
            fresh_ptr,
            acq_samples,
            fresh_window))
    {
        return false;
    }

    auto t0 =
        std::chrono::high_resolution_clock::now();

    fresh =
        acqMgr.runSingle(
            meta,
            fresh_ptr,
            acq_samples,
            (int)focusPRN);

    auto t1 =
        std::chrono::high_resolution_clock::now();

    double elapsed_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    printf("[*] Fresh acquisition PRN %d took %.1f ms\n",
           focusPRN,
           elapsed_ms);

    if (fresh.snr <= 9.0f)
    {
        printf("[!] Fresh acquisition failed PRN %d SNR %.1f\n",
               focusPRN,
               fresh.snr);
        return false;
    }

    printf(" FRESH | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n",
           fresh.prn,
           fresh.snr,
           fresh.bin,
           fresh.codePhase);

    return true;
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
    static int dbg_counter = 0;
    TrackingEngine tracking;

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
                    //                    printf("\n[RING OK] write=%llu\n", rx.get_write_index());
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
                    tracking.activeChannels.clear();

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
                        AcqResult fresh = {};
                        uint64_t fresh_cursor = 0;

                        if (!runFreshFocusedAcquisition(
                                rx,
                                acqMgr,
                                meta,
                                focusPRN,
                                ms_samples,
                                acq_samples,
                                fresh,
                                fresh_cursor))
                        {
                            printf("[!] Unable to refresh focused acquisition for PRN %d\n",
                                   focusPRN);
                            continue;
                        }

                        tracking.activeChannels.emplace_back(
                            fresh.prn,
                            (double)meta.fs_rate,
                            fresh,
                            pcs.getSV(fresh.prn));
                            
                            tracking.activeChannels.back()
                             .processor->setInputIsComplex(rx.input_is_complex());

                        for (int phase = 0; phase < 20; ++phase)
                        {
                            tracking.activeChannels.back().decoder[phase]->setFocus(false);
                        }

                        auto &state = tracking.activeChannels.front();

                        state.total_tracked_ms = 0;

                        // state.handover_sample_tick = (uint32_t)std::round(focusTarget->codePhase);
                        state.handover_sample_tick = (uint32_t)std::round(fresh.codePhase);
                        state.handover_unix_time = acq_ptr[0].unix_time;
                        state.sampleCursor = fresh_cursor + acq_samples;

                        // Important: add this field to ChannelState next.
                        uint64_t write = rx.get_write_index();
                        uint64_t aligned_write = write - (write % ms_samples);

                        state.sampleCursor = aligned_write - ms_samples;

                        acq_needed = false;

                        printf("[*] HANDOVER SUCCESS: fresh acquisition window [%llu, %llu)\n",
                               fresh_cursor,
                               fresh_cursor + acq_samples);
                        auto timing =
                            rx.get_timing_status(
                                fresh_cursor + acq_samples,
                                ms_samples);

                        printf(
                            "[*] Fresh handoff lag=%.1f ms margin=%.1f ms ring=%.1f ms\n",
                            timing.lag_ms,
                            timing.margin_ms,
                            timing.ring_ms);

                        continue;
                    }
                }

                continue;
            }
            /* Tracking goes here */
            tracking.step(rx, meta, focusPRN, out, acq_needed);

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
