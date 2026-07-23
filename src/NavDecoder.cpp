#include "NavDecoder.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

int32_t NavDecoder::extendSign(uint32_t val, int bits)
{
    if (val & (1 << (bits - 1)))
    {
        return val | (~0U << bits);
    }
    return val;
}

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
    _navTimerMs = 0;    // Initialize the NAV timer
    _sessionEpochs = 0; // Initialize the session epoch counter

    for (int i = 0; i < 20; i++)
    {
        _sync.histograms[i] = 0;
        _sync.buffer[i] = 0;
    }
}

void NavDecoder::processTrackingMetrics(const CorrelatorResult &metrics)
{
    // --- 1. ROBUST LOSS OF LOCK HANDLING ---
    if (!metrics.is_locked)
    {
        if (_bitSyncLocked || _sessionEpochs > 500)
        {
            _bitSyncLocked = false;
            _sessionEpochs = 0;
            _bitOffset = 0;
            _bitIntegrationI = 0.0;
            _last_symbol = 0;

            for (int i = 0; i < 20; i++)
            {
                _sync.histograms[i] = 0;
                _sync.buffer[i] = 0;
            }
        }
        return;
    }

    if (metrics.numSymbols <= 0)
        return;

    // --- INCREMENT ONCE PER 1MS TRACKING CALLBACK ---
    if (!_bitSyncLocked)
    {
        _sessionEpochs++;
    }

    for (const auto &epoch : metrics.epochs)
    {
        uint32_t current_phase = (uint32_t)(_decoderSampleCounter % 20);
        _decoderSampleCounter++;

        if (_bitSyncLocked)
        {
            if (current_phase == _bitOffset)
            {
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
            // Phase 1: FLL Noise Gate (First 1000 ms)
            if (_sessionEpochs < 1000)
            {
                _last_symbol = epoch.symbol;
                continue;
            }

            // Phase 2: Strict 5-Second Histogram Window (1000ms to 6000ms)
            if (_sessionEpochs < 6000)
            {
                if (_last_symbol != 0 && epoch.symbol != _last_symbol)
                {
                    _sync.histograms[current_phase]++;
                }
                _last_symbol = epoch.symbol;
            }
            // Phase 3: Lock Trigger (Fires precisely at 6000 ms)
            else if (!_bitSyncLocked && _sessionEpochs >= 6000)
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
                printf("\n[NAV] Sync Locked at offset %d (True robust max votes: %d)\n", _bitOffset, maxVal);
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
        if (_isInverted)
        {
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
    if (_isFocused)
    {
        printf("\n[NAV] Word %d Candidate Raw Bits: ", wordNum);
        for (int i = 29; i >= 0; i--)
        {
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

            if (_isFocused)
            {
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
            _parityFailCount = 0;
            _tempEphemeris = {};
            _hasSF1 = false;
            _hasSF2 = false;
            _hasSF3 = false;

            if (_isFocused)
            {
                printf("\n[NAV] Too many consecutive parity failures. Dropping frame sync and resetting counter.\n");
            }
        }
        else
        {
            if (_isFocused)
            {
                printf("\n[NAV] Parity failed for word %d (Failure count: %d), but maintaining frame sync...\n", wordNum, _parityFailCount);
            }
        }
        return false;
    }

    _parityFailCount = 0;
    _parityPassCount++;

    if (_isFocused)
    {
        printf("\n[NAV] Parity PASSED for Word %d!\n", wordNum);
    }

    // Payload is already normalized via global _isInverted correction
    uint32_t payloadWord = _shiftReg & 0x3FFFFFFF;

    // Extract the top 24 data bits (strip off the bottom 6 parity bits)
    uint32_t dataBits = (payloadWord >> 6) & 0xFFFFFF;

    // Buffer it (wordNum is 1-10, so index is 0-9)
    _subframeWords[wordNum - 1] = dataBits;

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

    // --- NEW: Trigger Subframe Decoding ---
    if (wordNum == 10)
    {
        decodeSubframe(_subframeID);
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

void NavDecoder::decodeSubframe(int subframeID)
{
    // IS-GPS-200 mandates Pi exactly as this:
    const double GPS_PI = 3.1415926535898;

    if (subframeID == 1)
    {
        _tempEphemeris.prn = _prn;

        // Word 3: Week Number (10 bits)
        _tempEphemeris.weekNumber = (_subframeWords[2] >> 14) & 0x3FF;

        // Word 8: t_oc (bottom 16 bits)
        _tempEphemeris.toc = (_subframeWords[7] & 0xFFFF) * 16;

        // Word 9: af2 (top 8 bits, signed)
        _tempEphemeris.af2 = extendSign((_subframeWords[8] >> 16) & 0xFF, 8) * pow(2, -55);

        // Word 9: af1 (bottom 16 bits, signed)
        _tempEphemeris.af1 = extendSign(_subframeWords[8] & 0xFFFF, 16) * pow(2, -43);

        // Word 10: af0 (top 22 bits, signed)
        _tempEphemeris.af0 = extendSign((_subframeWords[9] >> 2) & 0x3FFFFF, 22) * pow(2, -31);
        _hasSF1 = true;
    }
    else if (subframeID == 2)
    {
        // Word 3: Crs (16 bits, signed)
        _tempEphemeris.crs = extendSign(_subframeWords[2] & 0xFFFF, 16) * pow(2, -5);

        // Word 4: Delta n (16 bits, signed)
        _tempEphemeris.dn = extendSign(_subframeWords[3] >> 8, 16) * pow(2, -43) * GPS_PI;

        // Words 4 & 5: M0 (32 bits, signed)
        uint32_t m0_raw = ((_subframeWords[3] & 0xFF) << 24) | _subframeWords[4];
        _tempEphemeris.m0 = extendSign(m0_raw, 32) * pow(2, -31) * GPS_PI;

        // Word 6: Cuc (16 bits, signed)
        _tempEphemeris.cuc = extendSign(_subframeWords[5] >> 8, 16) * pow(2, -29);

        // Words 6 & 7: Eccentricity (32 bits, unsigned)
        uint32_t e_raw = ((_subframeWords[5] & 0xFF) << 24) | _subframeWords[6];
        _tempEphemeris.ecc = e_raw * pow(2, -33);

        // Word 8: Cus (16 bits, signed)
        _tempEphemeris.cus = extendSign(_subframeWords[7] >> 8, 16) * pow(2, -29);

        // Words 8 & 9: sqrt(A) (32 bits, unsigned)
        uint32_t sqrta_raw = ((_subframeWords[7] & 0xFF) << 24) | _subframeWords[8];
        _tempEphemeris.sqrta = sqrta_raw * pow(2, -19);

        // Word 10: toe (16 bits)
        _tempEphemeris.toe = (_subframeWords[9] >> 8) & 0xFFFF;
        _tempEphemeris.toe *= 16.0;
        _hasSF2 = true;
    }
    else if (subframeID == 3)
    {
        // Word 3: Cic (16 bits, signed)
        _tempEphemeris.cic = extendSign(_subframeWords[2] >> 8, 16) * pow(2, -29);

        // Words 3 & 4: Omega0 (32 bits, signed)
        uint32_t omega0_raw = ((_subframeWords[2] & 0xFF) << 24) | _subframeWords[3];
        _tempEphemeris.omega0 = extendSign(omega0_raw, 32) * pow(2, -31) * GPS_PI;

        // Word 5: Cis (16 bits, signed)
        _tempEphemeris.cis = extendSign(_subframeWords[4] >> 8, 16) * pow(2, -29);

        // Words 5 & 6: i0 (32 bits, signed)
        uint32_t i0_raw = ((_subframeWords[4] & 0xFF) << 24) | _subframeWords[5];
        _tempEphemeris.i0 = extendSign(i0_raw, 32) * pow(2, -31) * GPS_PI;

        // Word 7: Crc (16 bits, signed)
        _tempEphemeris.crc = extendSign(_subframeWords[6] >> 8, 16) * pow(2, -5);

        // Words 7 & 8: omega (Argument of Perigee)
        // Add the 24-bit mask (& 0xFFFFFF) to the lower word:
        uint32_t omega_raw = ((_subframeWords[6] & 0xFF) << 24) | (_subframeWords[7] & 0xFFFFFF);
        _tempEphemeris.omega = extendSign(omega_raw, 32) * pow(2, -31) * GPS_PI;

        // Word 9: Omega Dot (24 bits, signed)
        _tempEphemeris.omegaDot = extendSign(_subframeWords[8], 24) * pow(2, -43) * GPS_PI;

        // Word 10: IDOT (14 bits, signed)
        _tempEphemeris.iDot = extendSign(_subframeWords[9] >> 8, 14) * pow(2, -43) * GPS_PI;

        // --- SUBFRAME 3 COMPLETE ---
        _hasSF3 = true;
        if (_hasSF1 && _hasSF2 && _hasSF3)
        {
            _tempEphemeris.isValid = true;
            ConstellationManager::getInstance().commitEphemeris(_prn, _tempEphemeris);

            if (_isFocused)
            {
                printf("\n[NAV] PRN %2d | Ephemeris fully decoded & committed to ConstellationManager!\n", _prn);
                ConstellationManager::getInstance().printEphemerisSanityCheck(_prn);
            }

            // Optional: Reset flags if you want to force a fresh download every time,
            // but keeping them true allows continuous rapid updates as long as sync isn't lost.
        }
        else
        {
            if (_isFocused)
            {
                printf("\n[NAV] PRN %2d | Subframe 3 received, but missing SF1 or SF2. Waiting for next cycle...\n", _prn);
            }
        }
    }
}