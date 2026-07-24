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
#include "ConstellationManager.hpp"
#include "PVTSolver.hpp"
#include "Ephemeris.hpp"

// #define CAPTURE_TRACKING_DATA

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

    if (write < acq_samples + ms_samples)
        return false;

    uint64_t latest_complete_ms =
        write - (write % ms_samples);

    fresh_cursor =
        latest_complete_ms - acq_samples;

    RawSample *fresh_ptr = nullptr;
    std::vector<RawSample> fresh_window;

    if (!rx.get_window(
            fresh_cursor,
            fresh_ptr,
            (unsigned int)acq_samples,
            fresh_window))
    {
        return false;
    }

    uint32_t mod =
        fresh_ptr[0].sample_tick %
        (uint32_t)ms_samples;

    if (mod != 0)
    {
        if (fresh_cursor < mod)
            return false;

        fresh_cursor -= mod;

        if (!rx.get_window(
                fresh_cursor,
                fresh_ptr,
                (unsigned int)acq_samples,
                fresh_window))
        {
            return false;
        }

        mod =
            fresh_ptr[0].sample_tick %
            (uint32_t)ms_samples;
    }

    printf("[FRESH WIN] cursor=%llu tick=%u mod=%u samples=%zu\n",
           (unsigned long long)fresh_cursor,
           fresh_ptr[0].sample_tick,
           mod,
           acq_samples);

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
                uint32_t reacquirePrn = 0;

                if (tracking.popReacquire(reacquirePrn))
                {
                    AcqResult fresh = {};
                    uint64_t fresh_cursor = 0;

                    if (runFreshFocusedAcquisition(
                            rx,
                            acqMgr,
                            meta,
                            reacquirePrn,
                            ms_samples,
                            acq_samples,
                            fresh,
                            fresh_cursor))
                    {
                        tracking.beginTracking(
                            rx,
                            meta,
                            fresh,
                            fresh_cursor,
                            acq_samples);
                    }
                    else
                    {
                        printf("[!] Focused reacquire failed for PRN %u\n", reacquirePrn);
                    }

                    acq_needed = !tracking.hasActiveChannels();
                    continue;
                }

                if (tracking.hasActiveChannels())
                {
                    acq_needed = false;
                    continue;
                }

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
                    printf("[SURVEY WIN] cursor=%llu tick=%u mod=%u samples=%zu\n",
                           (unsigned long long)acq_cursor,
                           acq_ptr[0].sample_tick,
                           acq_ptr[0].sample_tick % (uint32_t)ms_samples,
                           acq_samples);
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

                        if (!tracking.beginTracking(
                                rx,
                                meta,
                                fresh,
                                fresh_cursor,
                                acq_samples))
                        {
                            printf("[!] Unable to begin tracking PRN %d\n", fresh.prn);
                            continue;
                        }

                        acq_needed = false;

                        printf("[*] HANDOVER SUCCESS: fresh acquisition window [%llu, %llu)\n",
                               (unsigned long long)fresh_cursor,
                               (unsigned long long)(fresh_cursor + acq_samples));

#ifdef CAPTURE_TRACKING_DATA
                        char capture_name[64];
                        snprintf(capture_name,
                                 sizeof(capture_name),
                                 "tracking_prn%03d",
                                 fresh.prn);

                        tracking.captureReplayPackage(
                            rx,
                            meta,
                            fresh,
                            fresh_cursor,
                            ms_samples,
                            100,
                            rx.input_is_complex(),
                            capture_name);
#endif

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
// =================================================================
            // --- PVT ENGINE CHECK ---
            // =================================================================
            static int pvtTimerMs = 0;
            pvtTimerMs += 1; // Assuming 1 iteration = ~1ms of processed data

            if (pvtTimerMs >= 1000) 
            {
                pvtTimerMs = 0; 

                if (ConstellationManager::getInstance().hasValidEphemeris(focusPRN)) 
                {
                    Ephemeris liveEph = ConstellationManager::getInstance().getEphemeris(focusPRN);

                    // --- 1. Get the High-Precision Transmit Time ---
                    // tracking.getCodePhase() should return the current chip offset (0.0 to 1023.0)
                    double exactTransmitTime = navDecoder.getExactTransmitTime(tracking.getCodePhase());

                    // --- 2. The TOE Guard ---
                    double timeSinceToe = exactTransmitTime - liveEph.toe;
                    
                    // Handle GPS week crossovers
                    if (timeSinceToe >  302400.0) timeSinceToe -= 604800.0;
                    if (timeSinceToe < -302400.0) timeSinceToe += 604800.0;

                    if (std::abs(timeSinceToe) > 7200.0) {
                        printf("\n[PVT WARN] PRN %d Ephemeris is stale (%.1f seconds old). Waiting for new subframes...\n", 
                               focusPRN, timeSinceToe);
                    }
                    else 
                    {
                        // --- 3. Fire the Math Engine ---
                        Vector3 satPos = PVTSolver::calculateSatPosition(liveEph, exactTransmitTime);

                        double radiusKm = std::sqrt(satPos.x * satPos.x + 
                                                    satPos.y * satPos.y + 
                                                    satPos.z * satPos.z) / 1000.0;

                        printf("\n=======================================================\n");
                        printf("[PVT ENGINE] LIVE SATELLITE POSITION (ECEF)\n");
                        printf("=======================================================\n");
                        printf(" PRN          : %d\n", focusPRN);
                        printf(" Transmit Time: %.6f sec (Precision)\n", exactTransmitTime);
                        printf(" TOE Age      : %.1f sec\n", timeSinceToe);
                        printf(" X Coordinate : %15.3f meters\n", satPos.x);
                        printf(" Y Coordinate : %15.3f meters\n", satPos.y);
                        printf(" Z Coordinate : %15.3f meters\n", satPos.z);
                        printf(" Orbit Radius : %15.3f km\n", radiusKm);
                        printf("=======================================================\n");
                    }
                }
            }
            // =================================================================
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
