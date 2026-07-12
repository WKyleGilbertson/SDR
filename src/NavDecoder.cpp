#include "NavDecoder.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

NavDecoder::NavDecoder(int prn, double fs) : _prn(prn), _fs_rate(fs), _subframeBuffer(300, 0)
{
    _msCounter = 0;
    _totalBitsCounter = 0; // Initialize these!
    _shiftReg64 = 0;
    _subframeBitIdx = 0;
    _wordCounter = 0;
    _isInverted = false;
    _msCounter = 0;
    _bitOffset = -1;
    _bitSyncLocked = false;
    _bitIntegrationI = 0.0;
    _msInBitCounter = 0;
    _consecutivePasses = 0;
    _decoderSampleCounter = 0;
    _d29Star = 0;
    _d30Star = 0;
    _shiftReg = 0;
    _shiftReg64 = 0;
    _frameSync = false;

    for (int i = 0; i < 20; i++)
    {
        _sync.histograms[i] = 0;
        _sync.buffer[i] = 0;
    }
}

void NavDecoder::processTrackingMetrics(const CorrelatorResult &metrics)
{
    if (!metrics.is_locked || metrics.numSymbols <= 0)
        return;

    for (const auto &epoch : metrics.epochs)
    {
        // 1. Calculate absolute millisecond index based on sample clock
        // This ensures we are locked to the hardware's sample timeline
        uint64_t ms_index = epoch.sample_tick / (uint64_t)(_fs_rate / 1000.0);

        // 2. Determine phase (0-19)
        uint32_t current_phase = (uint32_t)(ms_index % 20);

        // 3. Bit Synchronization Logic
        // Instead of a simple counter, use the phase to trigger integration
        // The _bitOffset is now our 'lock' point
        if (_bitSyncLocked)
        {
            if (current_phase == _bitOffset)
            {
                // We are at the start of a new 20ms bit block
                int8_t integratedBit = (_bitIntegrationI >= 0.0) ? 1 : -1;

                // Process the completed bit
                finalizeBit(integratedBit);

                // Reset for the next 20ms block
                _bitIntegrationI = (double)epoch.symbol;
            }
            else
            {
                // Continue integrating energy
                _bitIntegrationI += (double)epoch.symbol;
            }
        }
        else
        {
            // Bit Sync Search: Histogramming
            // Check for transitions at this specific phase
            static int64_t last_symbol = 0;
            if (last_symbol != 0 && epoch.symbol != last_symbol)
            {
                _sync.histograms[current_phase]++;
            }
            last_symbol = epoch.symbol;

            // Simple lock acquisition check
            if (_msCounter++ > 5000)
            {
                int maxVal = 0;
                for (int i = 0; i < 20; i++)
                {
                    if (_sync.histograms[i] > maxVal)
                    {
                        maxVal = _sync.histograms[i];
                        _bitOffset = i;
                    }
                }
                _bitSyncLocked = true;
                printf("[NAV] Sync Locked at offset %d\n", _bitOffset);
            }
        }
    }
}

void NavDecoder::finalizeBit(int8_t bit)
{
    uint32_t val = (bit > 0) ? 1 : 0;
    _shiftReg64 = (_shiftReg64 << 1) | val;
    _totalBitsCounter++;

    // 1. IF NOT SYNCED: Hunt for preamble
    if (!_frameSync)
    {
        uint8_t window = (uint8_t)(_shiftReg64 & 0xFF);
        if (window == 0x8B || window == 0x74)
        {
            _isInverted = (window == 0x74);
            _frameSync = true;
            _subframeBitIdx = 0;
            _wordCounter = 1;
            printf("\n[NAV] *** FRAME SYNC LOCKED (Polarity: %s) ***\n",
                   _isInverted ? "INVERTED" : "NORMAL");
        }
    }
    // 2. IF SYNCED: Do NOT look for preambles. Only process bits.
    else
    {
        processFramedBit(val);
    }
}

void NavDecoder::processBit(int8_t bit)
{
    std::vector<int8_t> oneBit;
    oneBit.push_back(bit);
    processBits(oneBit);
}

void NavDecoder::processBits(const std::vector<int8_t> &bits)
{
    // Kept intact for fallback unit tests processing pre-demodulated bit arrays
    for (int8_t bit : bits)
    {
        uint32_t bVal = (bit > 0) ? 1 : 0;
        _shiftReg = ((_shiftReg << 1) | bVal) & 0xFFFFFFFF;

        if (!_frameSync)
        {
            uint8_t fwdNormal = (_shiftReg & 0xFF);
            uint8_t fwdInverted = (~_shiftReg & 0xFF);

            bool match = false;
            if (fwdNormal == 0x8B)
            {
                match = true;
                _isInverted = false;
            }
            else if (fwdInverted == 0x8B)
            {
                match = true;
                _isInverted = true;
            }

            if (match)
            {
                _preambleCandidateCount++;
                _frameSync = true;
                _subframeBitIdx = 0;
                _consecutivePasses = 0;
                _shiftReg64 = 0;
            }
        }
        else
        {
            _subframeBitIdx++;
            if (_subframeBitIdx >= 30)
            {
                _subframeBitIdx = 0;
                static int wordCounter = 0;
                wordCounter = (wordCounter % 10) + 1;

                uint32_t workingWord = _shiftReg & 0x3FFFFFFF;
                if (_isInverted)
                    workingWord = ~workingWord & 0x3FFFFFFF;

                uint32_t originalRegBackup = _shiftReg;
                _shiftReg = workingWord;

                if (!handleWord(wordCounter))
                {
                    wordCounter = 0;
                    _frameSync = false;
                }
                _shiftReg = originalRegBackup;
            }
        }
    }
}

bool NavDecoder::isParityValid(uint32_t word, int lastD30)
{
    // Fall back to clean array mapping to eliminate hardware bit shift direction bugs
    uint32_t d[31];
    for (int i = 1; i <= 30; ++i)
    {
        d[i] = (word >> (30 - i)) & 1;
    }

    uint32_t p1 = d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23] ^ lastD30;
    uint32_t p2 = d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21] ^ d[24];
    uint32_t p3 = d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22];
    uint32_t p4 = d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23];
    uint32_t p5 = d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[21] ^ d[22] ^ d[24];
    uint32_t p6 = d[1] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[12] ^ d[13] ^ d[15] ^ d[16] ^ d[18] ^ d[19] ^ d[22] ^ d[23] ^ lastD30;

    return (p1 == d[25] && p2 == d[26] && p3 == d[27] && p4 == d[28] && p5 == d[29] && p6 == d[30]);
}

uint32_t NavDecoder::getBits(int startBit, int len)
{
    uint32_t mask = (1U << len) - 1;
    return (_shiftReg >> (30 - (startBit + len - 1))) & mask;
}

bool NavDecoder::handleWord(int wordNum)
{
    if (!isParityValid(_shiftReg, _d30Star))
    {
        _parityFailCount++;
        _consecutivePasses = 0;
        _frameSync = false;
        return false;
    }

    _parityPassCount++;
    _consecutivePasses++;

    if (wordNum == 1)
    {
        // Telemetry (TLM) subframe validation space
    }

    if (wordNum == 2)
    {
        uint32_t rawTOW = (_shiftReg >> 13) & 0x1FFFF;
        _tow = rawTOW * 6;

        _subframeID = (_shiftReg >> 10) & 0x07;

        if (_isFocused)
        {
            printf("\n[NAV] PRN %d | Verified Word %d | Subframe: %d | GPS TOW: %u sec (%.2f hours into week)\n",
                   _prn, wordNum, _subframeID, _tow, (double)_tow / 3600.0);
        }
    }

    _d29Star = (_shiftReg >> 1) & 1;
    _d30Star = _shiftReg & 1;
    return true;
}

void NavDecoder::processFramedBit(uint32_t bit)
{
    _subframeBuffer[_subframeBitIdx++] = bit;

    if (_subframeBitIdx >= 30)
    {
        _subframeBitIdx = 0;

        // --- ADD THIS LOGGING ---
        uint32_t currentWord = 0;
        for (int i = 0; i < 30; i++)
            currentWord |= (_subframeBuffer[i] << (29 - i));
// Debugging the raw bit stream
printf("[DEBUG] Raw Bits: ");
for(int i=0; i<30; i++) printf("%d", _subframeBuffer[i]);
printf("\n");
        bool valid = isParityValid(currentWord, _d30Star);
        printf("[PARITY] Word %d: %08X | Valid: %s\n", _wordCounter, currentWord, valid ? "YES" : "NO");

        handleWord(_wordCounter++);
        if (_wordCounter > 10)
        {
            if (_parityFailCount > 20) // If we have 20 failed words in a row
            {
                _isInverted = !_isInverted; // Flip polarity
                _parityFailCount = 0;       // Reset
                printf("[NAV] *** Parity failed threshold, flipping polarity to: %s ***\n",
                       _isInverted ? "INVERTED" : "NORMAL");
            }
        }
    }
}