#pragma once
#include <vector>
#include <cstdint>
#include "ChannelProcessor.h"

struct Observation {
    int prn;
    uint32_t tow;         // TOW from Word 2 transmit time
    double sampleOffset;  // Exactly which sample the preamble bit started on
    double codePhase;     // The fractional chip (sub-sample precision)
    bool isValid;
};

struct BitSync {
    int8_t buffer[20];      // Sliding window of 1ms symbols
    int count = 0;          // How many symbols we've seen
    int bestOffset = -1;    // The found bit-start (0-19)
    int histograms[20];     // Tracks sign-changes at each offset
};

class NavDecoder {
public:

    //NavDecoder(int prn) : _prn(prn), _subframeBuffer(300, 0) {}
    NavDecoder(int prn);
    void processTrackingMetrics(const CorrelatorResult& metrics);
    void processBits(const std::vector<signed char>& bits, double promptI, double promptQ);
    bool hasSync() const { return _frameSync; }
    uint32_t getTOW() const { return _tow; }
    int getSubframeID() const { return _subframeID; }
    Observation getLastObservation() const { return _lastObservation; }
    void setFocus(bool focused) { _isFocused = focused; }

private:
    bool _isFocused = false;
    bool handleWord(int wordNum);
    bool isParityValid(uint32_t word, int lastD30);
    int _prn;
    uint64_t _shiftReg64 = 0; // To store enough 1 ms samples to see a preamble
    int _msOffset = 0;        // Found bit-start (0-19)
    int _msCounter = 0;       // Counter to track 20 ms boundaries
    uint32_t _shiftReg = 0;
    bool _frameSync = false;
    bool _isInverted = false;
    bool _reversed = false;
    int _subframeBitIdx = 0;
    int _lastBitOfPrevWord = 0;
    int _consecutivePasses = 0;
    int _d29Star = 0; // The 29th bit of previous word
    int _d30Star = 0; // The 30th bit of previous word
    
    uint32_t _tow = 0;
    int _subframeID = 0;
    
    std::vector<uint8_t> _subframeBuffer;
    Observation _lastObservation = {0, 0, 0, 0, false};

    uint32_t getBits(int startBit, int len); 
    BitSync _sync = {};
    uint64_t _decoderSampleCounter = 0;
};