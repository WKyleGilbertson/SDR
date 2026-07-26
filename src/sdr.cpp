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
#include <set>
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
#include "PositionSolver.hpp"
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
    bool auto_mode = true;

    if (argc > 1 && argv[1] != nullptr)
    {
        focusPRN = (uint32_t)atoi(argv[1]);
        auto_mode = false;
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
        if (!rx.wait_for_telemetry(meta))
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
                    // printf("\n[RING OK] write=%llu\n", rx.get_write_index());
                }
            }

            const size_t TARGET_CHANNELS = 5;
            static uint32_t survey_timer = 0;

            // Increment survey timer if we have open slots.
            // If empty, trigger immediately by forcing timer to 5000.
            if (tracking.activeChannels.size() < TARGET_CHANNELS)
            {
                survey_timer++;
                if (tracking.activeChannels.empty())
                    survey_timer = 5000;
            }
            else
            {
                survey_timer = 0;
            }

            // Run a survey scan every ~5 seconds if we need more satellites
            if (survey_timer >= 5000)
            {
                survey_timer = 0;

                // 1. Drain the reacquire queue first
                uint32_t reacquirePrn = 0;
                while (tracking.popReacquire(reacquirePrn) && tracking.activeChannels.size() < TARGET_CHANNELS)
                {
                    bool already_tracking = false;
                    for (const auto &ch : tracking.activeChannels)
                    {
                        if (ch.prn == (int)reacquirePrn)
                            already_tracking = true;
                    }

                    if (!already_tracking)
                    {
                        AcqResult fresh = {};
                        uint64_t fresh_cursor = 0;
                        if (runFreshFocusedAcquisition(rx, acqMgr, meta, reacquirePrn, ms_samples, acq_samples, fresh, fresh_cursor))
                        {
                            tracking.beginTracking(rx, meta, fresh, fresh_cursor, acq_samples);
                        }
                    }
                }

                // 2. Perform a full sky survey if slots are still open
                if (tracking.activeChannels.size() < TARGET_CHANNELS)
                {
                    uint64_t newest = rx.get_write_index();

                    if (newest >= acq_samples + ms_samples)
                    {
                        uint64_t acq_cursor = newest - acq_samples;
                        RawSample *acq_ptr = nullptr;
                        std::vector<RawSample> acq_window;

                        if (rx.get_window(acq_cursor, acq_ptr, (unsigned int)acq_samples, acq_window))
                        {
                            uint32_t tick_mod = acq_ptr[0].sample_tick % (uint32_t)ms_samples;
                            if (tick_mod != 0)
                            {
                                acq_cursor -= tick_mod;
                                rx.get_window(acq_cursor, acq_ptr, (unsigned int)acq_samples, acq_window);
                            }

                            auto results = acqMgr.run(meta, acq_ptr, acq_samples);

                            if (!results.empty())
                            {
                                printf("\n[SURVEY] %zu Satellites Visible (> %.1f dB):\n", results.size(), ACQ_SNR_THRESHOLD_DB);
                                for (const auto &res : results)
                                {
                                    printf("  VISIBLE | PRN %3d | SNR %5.1f | Bin %3d | Code %9.4f\n",
                                           res.prn, res.snr, res.bin, res.codePhase);
                                }

                                std::set<int> tracked_prns;
                                for (const auto &ch : tracking.activeChannels)
                                {
                                    tracked_prns.insert(ch.prn);
                                }

                                // Filter out WAAS (PRN > 32) and satellites we are already tracking
                                std::vector<AcqResult> candidates;
                                for (const auto &res : results)
                                {
                                    if (res.prn <= 32 && tracked_prns.find(res.prn) == tracked_prns.end())
                                    {
                                        candidates.push_back(res);
                                    }
                                }

                                // Sort remaining by SNR
                                std::sort(candidates.begin(), candidates.end(), [](const AcqResult &a, const AcqResult &b)
                                          { return a.snr > b.snr; });

                                // Allocate candidates to open slots
                                size_t open_slots = TARGET_CHANNELS - tracking.activeChannels.size();
                                size_t to_add = (candidates.size() > open_slots) ? open_slots : candidates.size();

                                for (size_t i = 0; i < to_add; ++i)
                                {
                                    AcqResult fresh = {};
                                    uint64_t fresh_cursor = 0;

                                    if (runFreshFocusedAcquisition(rx, acqMgr, meta, candidates[i].prn, ms_samples, acq_samples, fresh, fresh_cursor))
                                    {
                                        tracking.beginTracking(rx, meta, fresh, fresh_cursor, acq_samples);
                                        printf("[+] ALLOCATED NEW TARGET: PRN %2d\n", fresh.prn);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            /* Tracking goes here */
            tracking.step(rx, meta, focusPRN, out, acq_needed);

            // =================================================================
            // --- PVT ENGINE CHECK ---
            // =================================================================
            static int pvtTimerMs = 0;
            pvtTimerMs += 1;

            if (pvtTimerMs >= 1000)
            {
                pvtTimerMs = 0;

                struct ValidChan
                {
                    int prn;
                    double transmitTime;
                    Ephemeris eph;
                };
                std::vector<ValidChan> validChans;

                // 1. Gather all channels that claim to have a valid time
                for (const auto &chan : tracking.activeChannels)
                {
                    if (chan.last_is_locked &&
                        chan.decoder &&
                        chan.decoder->hasSync() &&
                        chan.decoder->getTOW() > 0 &&
                        ConstellationManager::getInstance().hasValidEphemeris(chan.prn))
                    {
                        ValidChan vc;
                        vc.prn = chan.prn;
                        vc.transmitTime = tracking.getExactTransmitTime(chan.prn);
                        vc.eph = ConstellationManager::getInstance().getEphemeris(chan.prn);
                        validChans.push_back(vc);
                    }
                }

                // 2. Filter outliers: Find the largest cluster of synchronized satellites
                std::vector<ValidChan> bestCluster;
                for (size_t i = 0; i < validChans.size(); ++i)
                {
                    std::vector<ValidChan> currentCluster;
                    double refTime = validChans[i].transmitTime;

                    for (size_t j = 0; j < validChans.size(); ++j)
                    {
                        // Group satellites within 30ms of each other
                        if (std::abs(validChans[j].transmitTime - refTime) < 0.030)
                        {
                            currentCluster.push_back(validChans[j]);
                        }
                    }

                    if (currentCluster.size() > bestCluster.size())
                    {
                        bestCluster = currentCluster;
                    }
                }

                // 3. Execute PVT Engine if we have at least 4 synced satellites
                if (bestCluster.size() >= 4)
                {
                    printf("\n=======================================================\n");
                    printf("[PVT ENGINE] COLLECTING CLUSTER OBSERVATIONS\n");
                    printf("=======================================================\n");

                    std::vector<Vector3> satPositions;
                    std::vector<double> pseudoranges;

                    // Find max transmit time IN THE CLEAN CLUSTER to set receiver baseline
                    double maxTransmit = -1.0;
                    for (const auto &vc : bestCluster)
                    {
                        // 1. Calculate Satellite Clock Bias
                        double dt = vc.transmitTime - vc.eph.toc;
                        if (dt > 302400.0)
                            dt -= 604800.0;
                        if (dt < -302400.0)
                            dt += 604800.0;
                        double dt_sv = vc.eph.af0 + vc.eph.af1 * dt + vc.eph.af2 * dt * dt;

                        // 2. Correct the raw transmit time to true GPS system time
                        double t_tx_true = vc.transmitTime - dt_sv;

                        if (t_tx_true > maxTransmit)
                            maxTransmit = t_tx_true;
                    }

                    double referenceReceiveTime = maxTransmit + 0.070;

                    for (const auto &vc : bestCluster)
                    {
                        // Apply the exact same clock correction
                        double dt = vc.transmitTime - vc.eph.toc;
                        if (dt > 302400.0)
                            dt -= 604800.0;
                        if (dt < -302400.0)
                            dt += 604800.0;
                        double dt_sv = vc.eph.af0 + vc.eph.af1 * dt + vc.eph.af2 * dt * dt;
                        double t_tx_true = vc.transmitTime - dt_sv;

                        double timeSinceToe = t_tx_true - vc.eph.toe;
                        if (timeSinceToe > 302400.0)
                            timeSinceToe -= 604800.0;
                        if (timeSinceToe < -302400.0)
                            timeSinceToe += 604800.0;

                        if (std::abs(timeSinceToe) > 7200.0)
                        {
                            printf(" [WARN] PRN %2d Ephemeris stale (%.1f s old)\n", vc.prn, timeSinceToe);
                        }
                        else
                        {
                            // Calculate position at the TRUE transmit time
                            Vector3 satPos = PVTSolver::calculateSatPosition(vc.eph, t_tx_true);

                            // Calculate actual relative time of flight
                            double timeOfFlight = referenceReceiveTime - t_tx_true;
                            double pRange = timeOfFlight * PVTSolver::SPEED_OF_LIGHT;

                            // 3. Apply Sagnac Effect (Earth Rotation during TOF)
                            double theta = PVTSolver::WGS84_OMEGA_E * timeOfFlight;
                            double x_corr = satPos.x + satPos.y * theta;
                            double y_corr = -satPos.x * theta + satPos.y;
                            satPos.x = x_corr;
                            satPos.y = y_corr;

                            satPositions.push_back(satPos);
                            pseudoranges.push_back(pRange);

                            printf(" [+] PRN %2d | Transmit: %.6f | X: %11.0f Y: %11.0f Z: %11.0f\n",
                                   vc.prn, t_tx_true, satPos.x, satPos.y, satPos.z);
                        }
                    }

                    // ... [Keep existing solver execution code below this] ...

                    if (satPositions.size() >= 4)
                    {
                        PositionSolution sol = PositionSolver::computePosition(satPositions, pseudoranges);

                        if (sol.isValid)
                        {
                            printf("\n=======================================================\n");
                            printf("[PVT ENGINE] POSITION SOLUTION ACHIEVED!\n");
                            printf("=======================================================\n");
                            printf(" ECEF X: %15.3f meters\n", sol.ecefPosition.x);
                            printf(" ECEF Y: %15.3f meters\n", sol.ecefPosition.y);
                            printf(" ECEF Z: %15.3f meters\n", sol.ecefPosition.z);
                            printf(" GDOP  : %15.3f\n", sol.gdop);
                            printf("=======================================================\n\n");

                            GeodeticCoordinates geo = PositionSolver::ecefToLLA(sol.ecefPosition);

                            printf("\n=======================================================\n");
                            printf("[PVT ENGINE] POSITION SOLUTION ACHIEVED!\n");
                            printf("=======================================================\n");
                            printf(" Latitude: %12.4f° N\n", geo.latitudeDegrees);
                            printf(" Longitude: %12.4f° W\n", -geo.longitudeDegrees);
                            printf(" Altitude: %12.3f meters\n", geo.altitudeMeters);
                            printf(" GDOP    : %12.3f\n", sol.gdop);
                            printf("=======================================================\n");

                            printf("\n[*] Initial fix acquired. Exiting application.\n");
                            fclose(out);
                            exit(0);
                        }
                        else
                        {
                            printf("\n[PVT] Matrix solver diverged (Poor Geometry / GDOP)\n");
                        }
                    }
                }
                else if (validChans.size() > 0)
                {
                    // Find actual spread just for logging
                    double minT = 1e9, maxT = -1.0;
                    for (const auto &vc : validChans)
                    {
                        if (vc.transmitTime < minT)
                            minT = vc.transmitTime;
                        if (vc.transmitTime > maxT)
                            maxT = vc.transmitTime;
                    }
                    printf("\n[PVT] Waiting for cluster synchronization. Spread: %.3f sec (Valid Chans: %zu, Max Cluster: %zu)\n",
                           (maxT - minT), validChans.size(), bestCluster.size());
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
