#include "NavDecoder.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

NavDecoder::NavDecoder(int prn) : _prn(prn), _subframeBuffer(300, 0)
{
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
    {
        return;
    }

    for (int i = 0; i < metrics.numSymbols; ++i)
    {
        int8_t sym = metrics.symbols[i];
        _msCounter++;

        // =================================================================
        // PHASE 1: BIT SYNC ALIGNMENT (Accumulates phase transitions)
        // =================================================================
        static double prevPi = 0.0;
        if ((metrics.Pi > 0.0 && prevPi < 0.0) || (metrics.Pi < 0.0 && prevPi > 0.0))
        {
            uint32_t cellPhase = _msCounter % 20;
            _sync.histograms[cellPhase]++;
        }
        prevPi = metrics.Pi;

// Extend convergence to 4000ms to let tracking loops settle firmly
        if (!_bitSyncLocked && _msCounter == 4000)
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
            // Add a +1 ms phase offset modifier if your front-end sample clock has a constant lag
            _bitOffset = bestPhase; 
            _bitSyncLocked = true;
            
            printf("\n[INFO] PRN %d Bit Sync Firm Lock! Settled Phase: %d ms\n", _prn, _bitOffset);
        }

        // =================================================================
        // PHASE 2 & 3: SYNCHRONIZED WORD INTEGRATION & DECODING LAYER
        // =================================================================
        
        // Force alignment: Only start collecting energy if bit sync has initialized
        if (_bitSyncLocked)
        {
            _msInBitCounter++;
            _bitIntegrationI += sym; // Accumulate energy across the aligned 20ms block

            // We hit a true 20ms bit boundary milestone
            if (_msInBitCounter >= 20)
            {
                // Finalize the navigation bit state based on accumulated energy
                int8_t integratedBit = (_bitIntegrationI > 0.0) ? 1 : -1;
                
                // Pack bit state based on whether our framing stream is inverted or normal
                uint32_t packingBit = (_isInverted) ? (integratedBit < 0 ? 1 : 0)
                                                    : (integratedBit > 0 ? 1 : 0);

                // Reset block accumulators for the next bit
                _bitIntegrationI = 0.0;
                _msInBitCounter = 0;

                // Case A: Looking for Frame Sync using pristine, un-smeared 20ms bits
                // Shift the clean, synchronized bit into our history tracking register
                _shiftReg64 = (_shiftReg64 << 1) | packingBit;
                uint8_t cleanByte = (uint8_t)(_shiftReg64 & 0xFF);

                if (!_frameSync)
                {
                    // Unconditional diagnostic: print the sliding window to see if it's hitting
                    static int byteDiagnosticCounter = 0;
                    if (++byteDiagnosticCounter >= 10)
                    {
                        byteDiagnosticCounter = 0;
                        printf("[NAV DIAGNOSTIC] PRN %d sliding byte: 0x%02X\n", _prn, cleanByte);
                    }
                    
                    if (cleanByte == 0x8B) 
                    {
                        _frameSync = true;
                        _isInverted = false;
                        _subframeBitIdx = 0; // Reset counter so the NEXT 30 bits form Word 1
                        _preambleCandidateCount++;
                        printf("\n\n>>> SUCCESS: PRN %d Preamble Locked via 20ms Clean Stream! [NORMAL] <<<\n\n", _prn);
                    }
                    else if (cleanByte == 0x74) 
                    {
                        _frameSync = true;
                        _isInverted = true;
                        _subframeBitIdx = 0; // Reset counter so the NEXT 30 bits form Word 1
                        _preambleCandidateCount++;
                        printf("\n\n>>> SUCCESS: PRN %d Preamble Locked via 20ms Clean Stream! [INVERTED] <<<\n\n", _prn);
                    }
                }
                // Case B: Frame Sync is active, parse bits into sequential 30-bit words
                else
                {
                    _subframeBitIdx++;

                    // Check for preamble validation on the fly right when Word 10 finishes and Word 1 begins
                    static int wordCounter = 0;

                    if (wordCounter == 10 && _subframeBitIdx == 8)
                    {
                        // We are 8 bits into what should be the new subframe's TLM word (Preamble location)
                        uint8_t expectedPreamble = _isInverted ? 0x74 : 0x8B;
                        if (cleanByte == expectedPreamble)
                        {
                            if (_isFocused) printf("[NAV STATE] Sequential Subframe Validation PASSED at Word 10 boundary!\n");
                        }
                        else
                        {
                            if (_isFocused) printf("[NAV WARNING] Subframe Validation FAILED at boundary. Expected 0x%02X, got 0x%02X. Dropping lock.\n", expectedPreamble, cleanByte);
                            _frameSync = false;
                            wordCounter = 0;
                        }
                    }

                    if (_subframeBitIdx >= 30)
                    {
                        _subframeBitIdx = 0;
                        wordCounter = (wordCounter % 10) + 1;

                        uint32_t currentWord = (uint32_t)(_shiftReg64 & 0x3FFFFFFF);

                        // Context swap storage safely into standard telemetry handler pipeline
                        uint32_t originalRegBackup = _shiftReg;
                        _shiftReg = currentWord;

                        if (!handleWord(wordCounter))
                        {
                            if (_isFocused)
                            {
                                printf("[NAV] PRN %d Parity Fail Word %d. Dropping Sync.\n", _prn, wordCounter);
                            }
                            _frameSync = false;
                            wordCounter = 0;
                        }

                        _shiftReg = originalRegBackup;
                        // REMOVED: _shiftReg64 = 0; -> Let history slide naturally for preamble matching!
                    }
                }
            }
        }
        else
        {
            // Fallback / Pre-lock window: Visual character stream telemetry indicator
            if (_isFocused)
            {
                char navBitChar = (sym > 0) ? '#' : '-';
                printf("%c", navBitChar);

                static int lineCounter = 0;
                if (++lineCounter >= 60)
                {
                    lineCounter = 0;
                    lineCounter = 0;
                    printf(" |\n");
                }
                fflush(stdout);
            }
        }
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
            if (fwdNormal == 0x8B) { match = true; _isInverted = false; }
            else if (fwdInverted == 0x8B) { match = true; _isInverted = true; }

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
                if (_isInverted) workingWord = ~workingWord & 0x3FFFFFFF;

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