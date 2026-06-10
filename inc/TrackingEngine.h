#pragma once

#include <cstdint>
#include <cstdio>
#include <deque>
#include <list>
#include <memory>
#include <vector>

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

    std::deque<int8_t> epochSymbols;
    int nav20_sum = 0;
    int nav20_count = 0;

    uint64_t last_logged_sample_index = 0;
    uint64_t sampleCursor = 0;
    uint64_t total_tracked_ms = 0;

    uint32_t handover_sample_tick = 0;
    uint32_t handover_unix_time = 0;

    ChannelState(int p, double fs, const AcqResult& res, G2INIT s);
};

class TrackingEngine
{
public:
    std::list<ChannelState> activeChannels;

bool step(
    ElasticReceiver& rx,
    const RFE_Header_t& meta,
    uint32_t focusPRN,
    FILE* out,
    bool& acq_needed);

private:
    bool file_logging_enabled = true;
    uint64_t logged_ms = 0;
    static constexpr uint64_t max_logged_ms = 250;
};