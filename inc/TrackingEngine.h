#pragma once

#include <cstdint>
#include <cstdio>
#include <deque>
#include <list>
#include <memory>
#include <vector>
#include <cstddef>

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
    std::unique_ptr<NavDecoder> decoder[20];

    std::deque<int8_t> epochSymbols;
    int nav20_sum = 0;
    int nav20_count = 0;
    uint64_t nav20_groups = 0;
    uint64_t epoch_counter = 0;
    int nav_phase_sum[20] = {};
    int nav_phase_score[20] = {};
    int nav_phase_windows[20] = {};
    int64_t nav_phase_prompt_sum[20] = {};
    int64_t nav_phase_prompt_score[20] = {};
    int8_t nav_phase_prev_bit[20] = {};
    bool nav_phase_has_prev_bit[20] = {};
    uint32_t nav_phase_flip_count[20] = {};
    int nav_phase_best = -1;
    double nav_phase_ratio = 0.0;
    int8_t last_nav_bit = 0;
    double last_snr = 0.0;
    double last_doppler_hz = 0.0;
    double last_code_phase = 0.0;
    uint64_t last_logged_sample_index = 0;
    uint64_t sampleCursor = 0;
    uint64_t total_tracked_ms = 0;

    uint32_t handover_sample_tick = 0;
    uint32_t handover_unix_time = 0;

    ChannelState(int p, double fs, const AcqResult &res, G2INIT s);
};

class TrackingEngine
{
public:
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

private:
    void processEpoch(ChannelState &state, const EpochResult &epoch,
                      const RFE_Header_t &meta, FILE *out);
    void resetNavAccumulation(ChannelState &state);
    bool file_logging_enabled = true;

    uint64_t logged_ms = 0;
    static constexpr uint64_t max_logged_ms = 250;
    FILE *iq_log = nullptr;
    bool iq_log_header_written = false;
    uint64_t iq_log_rows = 0;
    static constexpr uint64_t max_iq_log_rows = 20000;
};