#include "NavDecoder.h"
#include <cmath>
#include <cstdio>

NavDecoder::NavDecoder(int prn) : _prn(prn), _subframeBuffer(300, 0)
{
    _msCounter = 0;
    _bitOffset = -1;
    _bitSyncLocked = false;
    _bitIntegrationI = 0.0;
    _msInBitCounter = 0;
    _consecutivePasses = 0;
    _decoderSampleCounter = 0;

    for (int i = 0; i < 20; i++)
    {
        _sync.histograms[i] = 0;
        _sync.buffer[i] = 0;
    }
}

void NavDecoder::processTrackingMetrics(const CorrelatorResult &metrics)
{
    if (!metrics.is_locked || metrics.numSymbols <= 0)
    {
        return;
    }

    for (int i = 0; i < metrics.numSymbols; ++i)
    {
        int8_t sym = metrics.symbols[i];
        _msCounter++;

        // 1. Bit Sync Alignment (Kept intact to capture absolute sample indices later)
        static double prevPi = 0.0;
        if ((metrics.Pi > 0.0 && prevPi < 0.0) || (metrics.Pi < 0.0 && prevPi > 0.0))
        {
            uint32_t cellPhase = _msCounter % 20;
            _sync.histograms[cellPhase]++;
        }
        prevPi = metrics.Pi;

        if (!_bitSyncLocked && _msCounter == 2000)
        {
            int maxFlips = 0;
            int bestPhase = 0;
            for (int b = 0; b < 20; ++b)
            {
                if (_sync.histograms[b] > maxFlips)
                {
                    maxFlips = _sync.histograms[b];
                    bestPhase = b;
                }
            }
            _bitOffset = bestPhase;
            _bitSyncLocked = true;
            if (_isFocused)
            {
                printf("\n[INFO] PRN %d Bit Sync Locked! Phase: %d ms\n", _prn, _bitOffset);
            }
        }

        // 2. LIVE CHARACTER DISPLAY (Prints your preferred format tracking lines)
        if (!_frameSync && _isFocused)
        {
            char navBitChar = (sym > 0) ? '#' : '-';
            printf("%c", navBitChar);

            static int lineCounter = 0;
            if (++lineCounter >= 60)
            {
                lineCounter = 0;
                printf(" |\n");
            }
            fflush(stdout);
        }

        // 3. FIX: 1 ms High-Resolution Sliding Preamble Search Engine
        // Convert the 1ms symbol directly to a binary bit and shift it into your window
        uint32_t bVal = (sym > 0) ? 1 : 0;
        _shiftReg = ((_shiftReg << 1) | bVal); // 32-bit register accumulates 1ms steps

        if (!_frameSync)
        {
            // Because bits last 20ms, an 8-bit preamble (10001011) spans exactly 160 ms steps.
            // We can isolate the true down-sampled 8-bit pattern by extracting every 20th step
            uint8_t extractedFwdByte = 0;
            uint8_t extractedInvByte = 0;

            for (int b = 0; b < 8; ++b)
            {
                // Read bits separated by exactly 20ms steps looking backwards in time
                uint32_t bitShiftOffset = b * 20;
                uint32_t isolatedBit = (_shiftReg >> bitShiftOffset) & 1;

                extractedFwdByte |= (isolatedBit << b);
                extractedInvByte |= ((isolatedBit ^ 1) << b); // Check inverted phase state
            }

            // Target GPS Preamble Constant: 0x8B
            if (extractedFwdByte == 0x8B || extractedInvByte == 0x8B)
            {
                _frameSync = true;
                _isInverted = (extractedInvByte == 0x8B);

                // Synchronize your word clock boundary indices directly to this exact millisecond step!
                _msInBitCounter = 0;
                _subframeBitIdx = 0;
                _consecutivePasses = 0;

                if (_isFocused)
                {
                    printf("\n\n>>> SUCCESS: PRN %d Preamble Locked via 1ms Array Match! [%s] <<<\n\n",
                           _prn, _isInverted ? "INVERTED" : "NORMAL");
                }
            }
        }
        // --- PHASE 2: WORD DECODING LAYER ---
        else
        {
            _msInBitCounter++;

            // Accumulate signal energy over the 20ms window
            _bitIntegrationI += metrics.Pi;

            if (_msInBitCounter >= 20)
            {
                // 1. Finalize the 20ms Navigation Bit
                int8_t integratedBit = (_bitIntegrationI > 0.0) ? 1 : -1;

                // 2. Resolve Inversion (Normalize based on Preamble phase found earlier)
                uint32_t packingBit = (_isInverted) ? (integratedBit < 0 ? 1 : 0)
                                                    : (integratedBit > 0 ? 1 : 0);

                // 3. Reset 20ms Integration Counters
                _bitIntegrationI = 0.0;
                _msInBitCounter = 0;

                // 4. Pack Normalized Bit into the 30-bit Word Register
                _shiftReg64 = (_shiftReg64 << 1) | packingBit;
                _subframeBitIdx++;

                // 5. Check for completed 30-bit Word
                if (_subframeBitIdx >= 30)
                {
                    _subframeBitIdx = 0;

                    // Track which word of the subframe we are on (1 through 10)
                    static int wordCounter = 0;
                    wordCounter = (wordCounter % 10) + 1;

                    // Prepare word for parity check (Keep original slider history separate)
                    uint32_t currentWord = (uint32_t)(_shiftReg64 & 0x3FFFFFFF);

                    // Backup the 1ms slider register to use its storage for the handler
                    uint32_t originalRegBackup = _shiftReg;
                    _shiftReg = currentWord;

                    if (!handleWord(wordCounter))
                    {
                        // Parity failed: Log and drop frame sync to find a new preamble
                        if (_isFocused)
                            printf("[NAV] PRN %d Parity Fail Word %d. Dropping Sync.\n", _prn, wordCounter);
                        _frameSync = false;
                        wordCounter = 0;
                    }

                    // Clean up registers for next word
                    _shiftReg = originalRegBackup;
                    _shiftReg64 = 0;
                }
            }
        }
    }
}

void NavDecoder::processBits(const std::vector<int8_t> &bits)
{
    for (int8_t bit : bits)
    {
        uint32_t bVal = (bit > 0) ? 1 : 0;

        // 1. Accumulate raw sliding data window parameters continuously
        _shiftReg = ((_shiftReg << 1) | bVal) & 0xFFFFFFFF;
        _shiftReg64 = (_shiftReg64 >> 1) | ((uint64_t)bVal << 63);

        // 2. Continuous preferred character terminal display logic block
        if (!_frameSync && _isFocused)
        {
            char navBitChar = (bit > 0) ? '#' : '-';
            printf("%c", navBitChar);

            static int continuousBitCount = 0;
            continuousBitCount++;
            if (continuousBitCount >= 60)
            {
                continuousBitCount = 0;
                printf(" | [Reg8: 0x%02X]\n", (_shiftReg & 0xFF));
            }
            fflush(stdout);
        }

        // --- PHASE 1: UNLOCKED PREAMBLE HUNTING ---
        if (!_frameSync)
        {
            uint8_t fwdNormal = (_shiftReg & 0xFF);
            uint8_t fwdInverted = (~_shiftReg & 0xFF);
            uint8_t revNormal = (_shiftReg64 >> 56) & 0xFF;
            uint8_t revInverted = (~_shiftReg64 >> 56) & 0xFF;

            bool match = false;
            if (fwdNormal == 0x8B)
            {
                match = true;
                _isInverted = false;
                _reversed = false;
            }
            else if (fwdInverted == 0x8B)
            {
                match = true;
                _isInverted = true;
                _reversed = false;
            }
            else if (revNormal == 0x8B)
            {
                match = true;
                _isInverted = false;
                _reversed = true;
            }
            else if (revInverted == 0x8B)
            {
                match = true;
                _isInverted = true;
                _reversed = true;
            }

            if (match)
            {
                _frameSync = true;
                _subframeBitIdx = 0; // FIX: Initialize bit count to 0 at the PREAMBLE boundary edge
                _consecutivePasses = 0;

                if (_isFocused)
                {
                    printf("\n\n>>> SUCCESS: PRN %d Preamble Locked! [%s] <<<\n",
                           _prn, _isInverted ? "INVERTED" : "NORMAL");
                }
            }
        }
        // --- PHASE 2: LOCKED FRAME WORD PACKING ---
        else
        {
            // Increment index ONLY when a valid preamble lock holds the frame alignment window open
            _subframeBitIdx++;

            if (_subframeBitIdx >= 30)
            {
                _subframeBitIdx = 0; // Clean reset matching a true 30-bit word milestone

                static int wordCounter = 0;
                wordCounter = (wordCounter % 10) + 1;

                uint32_t workingWord = _shiftReg;
                if (_reversed)
                {
                    workingWord = 0;
                    for (int b = 0; b < 30; ++b)
                    {
                        workingWord |= ((_shiftReg >> b) & 1) << (29 - b);
                    }
                }

                if (_isInverted)
                {
                    workingWord = ~workingWord;
                }

                uint32_t clean30BitWord = workingWord & 0x3FFFFFFF;

                // Backup register trace continuity safely before processing handleWord
                uint32_t originalRegBackup = _shiftReg;
                _shiftReg = clean30BitWord;

                bool valid = handleWord(wordCounter);
                if (!valid)
                {
                    wordCounter = 0;
                    _frameSync = false; // Parity failure: drop lock back to acquisition window safety
                }

                _shiftReg = originalRegBackup;
                _shiftReg = 0; // Clear slider register for next word channel block tracking loop
            }
        }
    }
}

bool NavDecoder::isParityValid(uint32_t word, int lastD30)
{
    // Extract individual bits using direct bit shifts (1-indexed matching GPS L1 ICD)
    // We bitwise XOR with lastD30 (D30*) where specified by the GPS parity algorithm
    int d25 = ((word >> 5) & 1) ^ (((word >> 23) & 1) ? 0 : lastD30); // Note: handles source sign inversion
    int d26 = ((word >> 4) & 1) ^ lastD30;
    int d27 = ((word >> 3) & 1) ^ lastD30;
    int d28 = ((word >> 2) & 1) ^ lastD30;
    int d29 = ((word >> 1) & 1) ^ lastD30;
    int d30 = (word & 1) ^ lastD30;

    // Compute the 6 parity check equations natively
    int p1 = ((word >> 29) & 1) ^ ((word >> 28) & 1) ^ ((word >> 27) & 1) ^ ((word >> 25) & 1) ^ ((word >> 23) & 1) ^ ((word >> 22) & 1) ^ ((word >> 20) & 1) ^ ((word >> 19) & 1) ^ ((word >> 18) & 1) ^ ((word >> 14) & 1) ^ ((word >> 10) & 1) ^ lastD30;
    int p2 = ((word >> 28) & 1) ^ ((word >> 27) & 1) ^ ((word >> 26) & 1) ^ ((word >> 24) & 1) ^ ((word >> 22) & 1) ^ ((word >> 21) & 1) ^ ((word >> 19) & 1) ^ ((word >> 18) & 1) ^ ((word >> 17) & 1) ^ ((word >> 13) & 1) ^ ((word >> 9) & 1);
    int p3 = ((word >> 29) & 1) ^ ((word >> 27) & 1) ^ ((word >> 26) & 1) ^ ((word >> 25) & 1) ^ ((word >> 23) & 1) ^ ((word >> 21) & 1) ^ ((word >> 20) & 1) ^ ((word >> 18) & 1) ^ ((word >> 17) & 1) ^ ((word >> 16) & 1) ^ ((word >> 12) & 1) ^ ((word >> 8) & 1);
    int p4 = ((word >> 28) & 1) ^ ((word >> 26) & 1) ^ ((word >> 25) & 1) ^ ((word >> 24) & 1) ^ ((word >> 22) & 1) ^ ((word >> 20) & 1) ^ ((word >> 19) & 1) ^ ((word >> 17) & 1) ^ ((word >> 16) & 1) ^ ((word >> 15) & 1) ^ ((word >> 11) & 1) ^ ((word >> 7) & 1);
    int p5 = ((word >> 28) & 1) ^ ((word >> 27) & 1) ^ ((word >> 25) & 1) ^ ((word >> 24) & 1) ^ ((word >> 23) & 1) ^ ((word >> 21) & 1) ^ ((word >> 19) & 1) ^ ((word >> 18) & 1) ^ ((word >> 16) & 1) ^ ((word >> 15) & 1) ^ ((word >> 14) & 1) ^ ((word >> 6) & 1) ^ lastD30;
    int p6 = ((word >> 29) & 1) ^ ((word >> 26) & 1) ^ ((word >> 24) & 1) ^ ((word >> 23) & 1) ^ ((word >> 20) & 1) ^ ((word >> 17) & 1) ^ ((word >> 14) & 1) ^ ((word >> 11) & 1) ^ ((word >> 10) & 1) ^ ((word >> 8) & 1) ^ ((word >> 7) & 1) ^ ((word >> 5) & 1) ^ lastD30;

    // Return true only if calculated parity bits perfectly match the word's transmitted parity bits
    return (p1 == d25 && p2 == d26 && p3 == d27 && p4 == d28 && p5 == d29 && p6 == d30);
}

/*bool NavDecoder::isParityValid(uint32_t word, int lastD30) {
    // Word is a 30-bit integer. Express parity checks using GPS L1 specifications
    // Parity mask coverage vectors (d25 through d30)
    uint32_t d[31];
    for (int i = 1; i <= 30; ++i) {
        d[i] = (word >> (30 - i)) & 1;
    }

    // Source values are XORed against the final bit of the preceding word if it was a 1
    uint32_t p1 = d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23] ^ lastD30;
    uint32_t p2 = d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21] ^ d[24];
    uint32_t p3 = d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22];
    uint32_t p4 = d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23];
    uint32_t p5 = d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[21] ^ d[22] ^ d[24];
    uint32_t p6 = d[1] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[12] ^ d[13] ^ d[15] ^ d[16] ^ d[18] ^ d[19] ^ d[22] ^ d[23] ^ lastD30;

    return (p1 == d[25] && p2 == d[26] && p3 == d[27] && p4 == d[28] && p5 == d[29] && p6 == d[30]);
} */

uint32_t NavDecoder::getBits(int startBit, int len)
{
    // Extracts 'len' bits starting from 'startBit' (1-indexed, matching GPS docs)
    uint32_t mask = (1U << len) - 1;
    return (_shiftReg >> (30 - (startBit + len - 1))) & mask;
}

bool NavDecoder::handleWord(int wordNum)
{
    // 1. Verify parity using the last bit of the previous word (stored in _d30Star)
    if (!isParityValid(_shiftReg, _d30Star))
    {
        _consecutivePasses = 0;
        _frameSync = false; // Parity failure! Drop frame lock to resync safety boundaries
        return false;
    }

    _consecutivePasses++;

    // 2. Decode WORD 1: Telemetry Word (TLM)
    if (wordNum == 1)
    {
        // Preamble is verified. Extract system flags if needed.
    }

    // 3. Decode WORD 2: Handover Word (HOW)
    if (wordNum == 2)
    {
        // GPS TOW is stored in bits 1 through 17 of the HOW word
        uint32_t rawTOW = (_shiftReg >> 13) & 0x1FFFF; // Shift past parity and subframe ID
        _tow = rawTOW * 6;                             // One TOW count equals exactly 6 seconds of absolute time

        // Subframe ID is stored in bits 20 through 22
        _subframeID = (_shiftReg >> 10) & 0x07;

        if (_isFocused)
        {
            printf("\n[NAV] PRN %d | Verified Word %d | Subframe: %d | GPS TOW: %u sec (%.2f hours into week)\n",
                   _prn, wordNum, _subframeID, _tow, (double)_tow / 3600.0);
        }
    }

    // 4. Update the parity dependency tracking states for the next word calculation pass
    _d29Star = (_shiftReg >> 1) & 1;
    _d30Star = _shiftReg & 1;
    return true;
}
