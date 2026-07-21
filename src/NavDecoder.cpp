#include "NavDecoder.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

NavDecoder::NavDecoder(int prn, double fs) : _prn(prn), _fs_rate(fs), _subframeBuffer(300, 0)
{
    _last_symbol = 0;
    _msCounter = 0;
    _totalBitsCounter = 0; // Initialize these!
    _shiftReg64 = 0;
    _subframeBitIdx = 0;
    _wordCounter = 0;
    _isInverted = false;
    _bitOffset = 0;
    _bitSyncLocked = false;
    _bitIntegrationI = 0.0;
    _msInBitCounter = 0;
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
        uint32_t current_phase = (uint32_t)(_decoderSampleCounter % 20);
        _decoderSampleCounter++; 

        if (_bitSyncLocked)
        {
            if (current_phase == _bitOffset)
            {
                // We are at the start of a new 20ms bit block
                int8_t integratedBit = (_bitIntegrationI >= 0.0) ? 1 : -1;
                processBit(integratedBit);
                _bitIntegrationI = (double)epoch.symbol;
            }
            else
            {
                _bitIntegrationI += (double)epoch.symbol;
            }
        }
        else
        {
            // --- FLL NOISE GATE ---
            // Completely ignore symbol transitions while the FLL is pulling in
            if (_decoderSampleCounter < 1000)
            {
                _last_symbol = epoch.symbol;
                continue; 
            }

            // Bit Sync Search: Histogramming
            if (_last_symbol != 0 && epoch.symbol != _last_symbol)
            {
                _sync.histograms[current_phase]++;
            }
            _last_symbol = epoch.symbol;

            // Lock acquisition check (requires 5000 clean ms of PLL data)
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
                printf("\n[NAV] Sync Locked at offset %d\n", _bitOffset);
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
    for (int8_t bit : bits)
    {
        uint32_t bVal = (bit > 0) ? 1 : 0;

        // Apply global stream inversion if active
        if (_isInverted) {
            bVal = !bVal;
        }

        // Push corrected raw bit into shift register
        _shiftReg = ((_shiftReg << 1) | bVal) & 0xFFFFFFFF;

        if (!_frameSync)
        {
            uint8_t fwdNormal = (_shiftReg & 0xFF);

            // Search for BOTH normal (0x8B) and inverted (0x74) preambles
            if (fwdNormal == 0x8B || fwdNormal == 0x74)
            {
                _preambleCandidateCount++;
                _frameSync = true;
                _subframeBitIdx = 8;

                // Set global stream inversion based on preamble type
                _isInverted = (fwdNormal == 0x74);
                _d30Star = _isInverted ? 1 : 0;
                
                if (_isFocused)
                {
                    printf("\n[NAV] Preamble Candidate (0x%02X) found! Polarity: %s\n", 
                           fwdNormal, _isInverted ? "INVERTED" : "NORMAL");
                }
            }
        }
        else
        {
            _subframeBitIdx++;
            if (_subframeBitIdx >= 30)
            {
                _subframeBitIdx = 0;
                _wordCounter = (_wordCounter % 10) + 1;

                // Evaluate the completed 30-bit word in the shift register
                if (!handleWord(_wordCounter))
                {
                    if (!_frameSync)
                    {
                        _wordCounter = 0;
                    }
                }
            }
        }
    }
}

bool NavDecoder::handleWord(int wordNum)
{
    #ifdef DEBUG_NAV
    if (_isFocused) {
        printf("\n[NAV] Word %d Candidate Raw Bits: ", wordNum);
        for (int i = 29; i >= 0; i--) {
            printf("%d", (_shiftReg >> i) & 1);
        }
        printf("\n");
    } 
    #endif

    // 1. Try checking parity with current state
    bool valid = isParityValid(_shiftReg, _d29Star, _d30Star);

    // 2. If it fails, test the opposite polarity
    if (!valid)
    {
        uint32_t invertedWord = ~_shiftReg & 0x3FFFFFFF;
        if (isParityValid(invertedWord, !_d29Star, !_d30Star))
        {
            // Permanently flip global stream polarity state
            _isInverted = !_isInverted;
            _d30Star = !_d30Star;
            _d29Star = !_d29Star;
            
            // Preserve top bits of shift register while correcting the lower 30 bits
            _shiftReg = (_shiftReg & 0xC0000000) | invertedWord;
            valid = true;
            
            if (_isFocused) {
                printf("\n[NAV] Polarity flip detected! Permanently flipping stream polarity.\n");
            }
        }
    }

    if (!valid)
    {
        _parityFailCount++;
        
        if (_parityFailCount > 3) 
        {
            _frameSync = false;
            if (_isFocused) {
                printf("\n[NAV] Too many consecutive parity failures (%d). Dropping frame sync.\n", _parityFailCount);
            }
        }
        else 
        {
            if (_isFocused) {
                printf("\n[NAV] Parity failed for word %d (Failure count: %d), but maintaining frame sync...\n", wordNum, _parityFailCount);
            }
        }
        return false;
    }

    _parityFailCount = 0;
    _parityPassCount++;

    if (_isFocused) {
        printf("\n[NAV] Parity PASSED for Word %d!\n", wordNum);
    }

    // Payload is already normalized via global _isInverted correction
    uint32_t payloadWord = _shiftReg & 0x3FFFFFFF;

    if (wordNum == 2)
    {
        uint32_t rawTOW = (payloadWord >> 13) & 0x1FFFF;
        _tow = rawTOW * 6;
        _subframeID = (payloadWord >> 8) & 0x07;

        if (_isFocused)
        {
            printf("\n[NAV] PRN %d | Verified Word %d | Subframe: %d | GPS TOW: %u sec (%.2f hours into week)\n",
                   _prn, wordNum, _subframeID, _tow, (double)_tow / 3600.0);
        }
    }

    // Update D29* and D30* for the next word
    _d29Star = (_shiftReg >> 1) & 1;
    _d30Star = _shiftReg & 1; 
    
    return true;
}

bool NavDecoder::isParityValid(uint32_t word, int lastD29, int lastD30)
{
    // Map bits 1-30 cleanly to an array
    uint32_t d[31];
    for (int i = 1; i <= 30; ++i)
    {
        d[i] = (word >> (30 - i)) & 1;
    }

    // IS-GPS-200 Corrected Parity Equations
    uint32_t p1 = lastD29 ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23];
    uint32_t p2 = lastD30 ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21] ^ d[24];
    uint32_t p3 = lastD29 ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22];
    uint32_t p4 = lastD30 ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23];
    uint32_t p5 = lastD30 ^ d[1] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[21] ^ d[22] ^ d[24];
    uint32_t p6 = lastD29 ^ d[3] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[11] ^ d[13] ^ d[15] ^ d[19] ^ d[22] ^ d[23] ^ d[24];

    // Validate the 6 computed parity bits against the 6 received parity bits (D25 - D30)
    return (p1 == d[25] && p2 == d[26] && p3 == d[27] && p4 == d[28] && p5 == d[29] && p6 == d[30]);
}

void NavDecoder::processFramedBit(uint32_t bit)
{
    fprintf(stdout, "\n[NAV] PRN %3d | Received Bit: %2d | Word Counter: %d | Subframe Bit Index: %d\n",
            _prn, bit, _wordCounter, _subframeBitIdx);
    _subframeBuffer[_subframeBitIdx++] = bit;

    if (_subframeBitIdx >= 30)
    {
        _subframeBitIdx = 0;

        // --- ADD THIS LOGGING ---
        uint32_t currentWord = 0;
        for (int i = 0; i < 30; i++)
            currentWord |= (_subframeBuffer[i] << (29 - i));
        // Debugging the raw bit stream
        /*
        printf("[DEBUG] Raw Bits: ");
        for (int i = 0; i < 30; i++)
            printf("%d", _subframeBuffer[i]);
        printf("\n"); */
        bool valid = isParityValid(currentWord, _d29Star, _d30Star);
        printf("[PARITY] Word %d: %08X | Valid: %s\n", _wordCounter, currentWord, valid ? "YES" : "NO");

        handleWord(_wordCounter++);
        if (_wordCounter > 10)
        {
            if (_parityFailCount > 20) // If we have 20 failed words in a row
            {
                _isInverted = !_isInverted; // Flip polarity
                _parityFailCount = 0;       // Reset
                printf("\n[NAV] *** Parity failed threshold, flipping polarity to: %s ***\n",
                       _isInverted ? "INVERTED" : "NORMAL");
            }
        }
    }
}