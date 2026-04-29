#pragma once
#include <vector>
#include <cstdint>
#include "PCSEngine.hpp"        // Contains AcqResult and unpackL1IF
#include "ElasticReceiver.h"    // Provides RFE_Header_t

class AcquisitionMgr {
private:
    PCSEngine& m_pcs;
    const size_t FFT_SIZE = 16384;
    const int NUM_MS = 5;

public:
    AcquisitionMgr(PCSEngine& engine) : m_pcs(engine) {}

    /**
     * Executes acquisition using the block's specific metadata.
     * Passing meta by const reference ensures we don't lose track of 
     * critical timing info like sample_tick.
     */
    std::vector<AcqResult> run(const RFE_Header_t& meta, uint8_t* raw_block);
};
