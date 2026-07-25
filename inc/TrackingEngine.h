#pragma once

#include <cstdint>
#include <cstdio>
#include <deque>
#include <list>
#include <memory>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <immintrin.h>

#include "ElasticReceiver.h"
#include "ChannelProcessor.h"
#include "NavDecoder.h"
#include "PCSEngine.hpp"
#include "g2init.h"

struct ChannelState
{
    int prn;
    AcqResult result;
    G2INIT sv;

    std::unique_ptr<ChannelProcessor> processor;
    std::unique_ptr<NavDecoder> decoder;
    std::vector<RawSample> ms_window_buffer;
    std::deque<int8_t> epochSymbols;
    uint64_t epoch_counter = 0;
    int8_t last_nav_bit = 0;
    double last_snr = 0.0;
    double last_doppler_hz = 0.0;
    double last_code_phase = 0.0;
    uint64_t last_logged_sample_index = 0;
    uint64_t sampleCursor = 0;
    uint64_t total_tracked_ms = 0;

    uint32_t handover_sample_tick = 0;
    uint32_t handover_unix_time = 0;
    uint32_t badLockEpochs = 0;

    double last_carrier_nco_hz = 0.0;
    bool last_is_locked = false;

    double total_correlator_time_us = 0.0;
    uint64_t timing_epochs_measured = 0;

    ChannelState(int p, double fs, const AcqResult &res, G2INIT s);
};

struct ChannelTask {
    ChannelState* state;
    RawSample* ms_ptr;
    int feed_samples;
    bool process_nav; // Flag to tell the main thread if NavDecoder needs processing
    CorrelatorResult* out_res;
};

class TrackingEngine
{
public:
    TrackingEngine();
    ~TrackingEngine();
    std::list<ChannelState> activeChannels;
    bool beginTracking(
        ElasticReceiver &rx,
        const RFE_Header_t &meta,
        const AcqResult &pcs_acq,
        uint64_t acq_cursor,
        size_t acq_samples);
    bool step(ElasticReceiver &rx, const RFE_Header_t &meta, uint32_t focusPRN,
              FILE *out, bool &acq_needed);
    bool captureReplayPackage(
        ElasticReceiver &rx,
        const RFE_Header_t &meta,
        const AcqResult &fresh,
        uint64_t fresh_cursor,
        size_t ms_samples,
        size_t capture_ms,
        bool input_is_complex,
        const char *basename);
    bool hasActiveChannels() const
    {
        return !activeChannels.empty();
    }

    void queueReacquire(uint32_t prn)
    {
        if (std::find(reacquireQueue.begin(), reacquireQueue.end(), prn) ==
            reacquireQueue.end())
        {
            reacquireQueue.push_back(prn);
        }
    }

    bool popReacquire(uint32_t &prn)
    {
        if (reacquireQueue.empty())
            return false;

        prn = reacquireQueue.front();
        reacquireQueue.pop_front();
        return true;
    }
    // Gets the nanosecond-precision transmit time from a specific tracking channel
    double getExactTransmitTime(int prn);

private:
    void processEpoch(ChannelState &state, const EpochResult &epoch,
                      const RFE_Header_t &meta, FILE *out);
    void resetNavAccumulation(ChannelState &state);
    std::deque<uint32_t> reacquireQueue;
    bool file_logging_enabled = true;

    uint64_t logged_ms = 0;
    static constexpr uint64_t max_logged_ms = 250;
    FILE *iq_log = nullptr;
    bool iq_log_header_written = false;
    uint64_t iq_log_rows = 0;
    static constexpr uint64_t max_iq_log_rows = 20000;

    // Thread Pool Infrastructure
    std::vector<std::thread> _workers;
    std::queue<ChannelTask> _task_queue;
    std::mutex _queue_mtx;
    std::condition_variable _cv;
    std::atomic<int> _pending_tasks{0};
    std::atomic<bool> _stop_pool{false};

    // Synchronization for the main thread to wait until all 1ms tasks finish
//    std::atomic<bool> _work_ready{false};
//    std::condition_variable _cv;
//    std::mutex _done_mtx;
//    std::condition_variable _done_cv;

    void workerLoop();
};