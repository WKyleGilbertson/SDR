#pragma once
#include <vector>
#include <cstdint>
#include "ChannelProcessor.h" // Access to CorrelatorResult definition

struct Observation
{
    int prn;
    uint32_t tow;
    double sampleOffset;
    double codePhase;
    bool isValid;
};

// Fixed array bracket template notation fields
struct BitSync
{
    int8_t buffer[20];
    int count = 0;
    int bestOffset = -1;
    int histograms[20];
};

class NavDecoder
{
public:
    NavDecoder(int prn, double fs);
    void processTrackingMetrics(const CorrelatorResult &metrics);
    void processBits(const std::vector<int8_t> &bits);
    void processBit(int8_t bit);
    bool hasSync() const { return _frameSync; }
    uint32_t getTOW() const { return _tow; }
    int getSubframeID() const { return _subframeID; }
    Observation getLastObservation() const { return _lastObservation; }
    void setFocus(bool focused) { _isFocused = focused; }
    int getPreambleCandidateCount() const { return _preambleCandidateCount; }
    int getParityPassCount() const { return _parityPassCount; }
    int getParityFailCount() const { return _parityFailCount; }
    void processFramedBit(uint32_t bit); // You'll need to define this or map it to handleWord

private:
    double _fs_rate;             // Needed to calculate ms from samples
    
    void finalizeBit(int8_t bit);
    // ... rest of your existing members
    bool _isFocused = false;
    int _prn;

    // Bit sync variables
    uint32_t _msCounter = 0;
    BitSync _sync = {};
    uint64_t _decoderSampleCounter = 0;
    int _bitOffset = -1;
    bool _bitSyncLocked = false;
    double _bitIntegrationI = 0.0;
    int _msInBitCounter = 0;

    // Frame Sync & Word structural pipeline variables
    uint64_t _shiftReg64 = 0;
    uint32_t _shiftReg = 0;
    bool _frameSync = false;
    bool _isInverted = false;
    bool _reversed = false;
    int _subframeBitIdx = 0;
    int _lastBitOfPrevWord = 0;

    // FIX: Restored missing compilation fields here
    int _consecutivePasses = 0;

    int _d29Star = 0;
    int _d30Star = 0;
    uint32_t _tow = 0;
    int _subframeID = 0;
    std::vector<uint8_t> _subframeBuffer;
    Observation _lastObservation = {0, 0, 0, 0, false};
    bool handleWord(int wordNum);
    bool isParityValid(uint32_t word, int lastD30);
    uint32_t getBits(int startBit, int len);
    int _preambleCandidateCount = 0;
    int _parityPassCount = 0;
    int _parityFailCount = 0;
    uint32_t _wordCounter = 0; // Track GPS subframe word indices (1 to 10.)
    int _totalBitsCounter = 0;
    int _lastPreambleBitLocation = -3000; // Initialize well out of range
};
