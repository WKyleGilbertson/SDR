#include "NavDecoder.h"
// NavDecoder.cpp
NavDecoder::NavDecoder(int prn) : _prn(prn) {
    _frameSync = false;
    _reversed = false;
    _isInverted = false;
    _subframeBitIdx = 0;
    _d30Star = 0;
    _subframeBuffer.resize(300);
}

void NavDecoder::processBits(const std::vector<signed char>& bits1ms, double promptI, double promptQ) {
    for (signed char b : bits1ms) {
        uint8_t bit = (b > 0) ? 1 : 0;
        
        // 1. Always update the 64-bit history of 1ms symbols
        _shiftReg64 = (_shiftReg64 << 1) | bit;

        if (!_frameSync) {
            // SEARCH MODE: Check all 20 possible millisecond offsets
            for (int offset = 0; offset < 20; offset++) {
                uint32_t pattern = 0;
                
                // Build an 8-bit word by sampling every 20th millisecond
                for (int i = 0; i < 8; i++) {
                    // Sample back: i=0 is most recent bit, i=7 is oldest
                    uint8_t sampledBit = (_shiftReg64 >> (i * 20 + offset)) & 1;
                    pattern |= (sampledBit << (7 - i)); 
                }

                // Check for Preamble 0x8B (or inverted/reversed)
                if (pattern == 0x8B || pattern == 0x74 || pattern == 0xD1 || pattern == 0x2E) {
                    _frameSync = true;
                    _msOffset = 19 - offset; // Align our counter to this edge
                    _msCounter = 0;
                    _subframeBitIdx = 0;
                    _isInverted = (pattern == 0x74 || pattern == 0x2E);
                    _reversed   = (pattern == 0xD1 || pattern == 0x2E);

                    // D30* is the bit just before the preamble (20ms further back)
                    _d30Star = (_shiftReg64 >> (8 * 20 + offset)) & 1;
                    if (_isInverted) _d30Star = !_d30Star;

                    // Fill the buffer with the 8 preamble bits we just validated
                    for (int i = 0; i < 8; i++) {
                        uint8_t pBit = (_shiftReg64 >> ((7 - i) * 20 + offset)) & 1;
                        _subframeBuffer[i] = _isInverted ? !pBit : pBit;
                    }
                    _subframeBitIdx = 8;
                    
                    if (_isFocused) printf("[PRN %2d] SYNC FOUND at offset %d\n", _prn, _msOffset);
                    break; 
                }
            }
        } else {
            // TRACKING MODE: Integrate 1ms samples into 20ms bits
            // We only "keep" the sample that lands on our discovered offset
            if (_msCounter == _msOffset) {
                _subframeBuffer[_subframeBitIdx++] = _isInverted ? !bit : bit;

                if (_subframeBitIdx == 300) {
                    // We have a full subframe. Process all 10 words.
                    for (int w = 1; w <= 10; w++) {
                        if (!handleWord(w)) {
                            _frameSync = false; // Parity failed, drop sync
                            break;
                        }
                    }
                    _subframeBitIdx = 0;
                    // Keep _frameSync true to look for the next one immediately
                }
            }
            _msCounter = (_msCounter + 1) % 20;
        }
    }
}

bool NavDecoder::handleWord(int wordNum) {
    uint32_t currentWord = 0;
    int startIdx = (wordNum - 1) * 30;

    // Correct Left-to-Right bit extraction
    for (int i = 0; i < 30; i++) {
        currentWord = (currentWord << 1) | (_subframeBuffer[startIdx + i] & 1);
    }

    if (_reversed) {
        uint32_t mirrored = 0;
        for (int i = 0; i < 30; i++) {
            if ((currentWord >> i) & 1) mirrored |= (1 << (29 - i));
        }
        currentWord = mirrored;
    }

    // Diagnostic Print for Focus PRN
    if (_isFocused && wordNum <= 2) {
        printf("[PRN %2d] W%d Raw: 0x%08X | D30:%d ", _prn, wordNum, currentWord, _d30Star);
    }

    if (!isParityValid(currentWord, _d30Star)) {
        if (wordNum == 1 && isParityValid(currentWord, _d30Star ^ 1)) {
            if (_isFocused) printf("-> RECOVERED\n");
            _d30Star ^= 1;
        } else {
            if (_isFocused) printf("-> FAIL\n");
            return false;
        }
    } else {
        if (_isFocused && wordNum <= 2) printf("-> OK\n");
    }

    // Capture bit 30 for the next word's parity check
    _d30Star = currentWord & 1;

    if (wordNum == 2) {
        // Extract Time of Week (TOW is 17 bits starting at bit 1 of HOW)
        uint32_t rawTow = (currentWord >> 13) & 0x1FFFF;
        uint32_t rawId  = (currentWord >> 8) & 0x07;
        _tow = rawTow * 6;
        if (_isFocused) {
            printf(" [PRN %2d] *** SUCCESS *** TOW: %d | Subframe: %d\n", _prn, _tow, rawId);
        }
    }

    return true;
}

uint32_t NavDecoder::getBits(int startBit, int len)
{
  if (startBit + len >= 300)
    return 0; // Safety gate
  uint32_t val = 0;
  for (int i = 0; i < len; i++)
  {
    val = (val << 1) | (_subframeBuffer[startBit + i] & 0x01);
  }
  return val;
}

bool NavDecoder::isParityValid(uint32_t word, int lastD30) {
    // word is 30 bits. Extract raw data bits d1 through d24
    uint32_t d = (word >> 6) & 0xFFFFFF;
    
    // If the last word's bit 30 was 1, the satellite inverted these bits
    if (lastD30 & 1) {
        d ^= 0xFFFFFF;
    }

    // Map d1-d24 into an array (1-indexed to match ICD-GPS-200 notation)
    int bits[31]; // Array declaration: int bits
    for (int i = 1; i <= 24; i++) {
        // Index: bits[i]
        bits[i] = (d >> (24 - i)) & 1; 
    }

    // Current word's received parity bits (d25-d30)
    int p_rx[7]; // Array declaration: int p_rx
    for (int i = 1; i <= 6; i++) {
        // Index: p_rx[i]
        p_rx[i] = (word >> (6 - i)) & 1; 
    }

    int d30star = lastD30 & 1;

    // The 6 Parity Equations from ICD-GPS-200 Table 20-XIV
    int p[7]; // Array declaration: int p

    // Note: All indices below like bits(1) must be bits
    p[1] = d30star ^ bits[1] ^ bits[2] ^ bits[3] ^ bits[5] ^ bits[6] ^ bits[10] ^ bits[11] ^ bits[12] ^ bits[13] ^ bits[14] ^ bits[17] ^ bits[18] ^ bits[20] ^ bits[23];
    p[2] = d30star ^ bits[2] ^ bits[3] ^ bits[4] ^ bits[6] ^ bits[7] ^ bits[11] ^ bits[12] ^ bits[13] ^ bits[14] ^ bits[15] ^ bits[18] ^ bits[19] ^ bits[21] ^ bits[24];
    p[3] = d30star ^ bits[1] ^ bits[3] ^ bits[4] ^ bits[5] ^ bits[7] ^ bits[8] ^ bits[12] ^ bits[13] ^ bits[14] ^ bits[15] ^ bits[16] ^ bits[19] ^ bits[20] ^ bits[22];
    p[4] = d30star ^ bits[2] ^ bits[4] ^ bits[5] ^ bits[6] ^ bits[8] ^ bits[9] ^ bits[13] ^ bits[14] ^ bits[15] ^ bits[16] ^ bits[17] ^ bits[20] ^ bits[21] ^ bits[23];
    p[5] = d30star ^ bits[1] ^ bits[3] ^ bits[5] ^ bits[6] ^ bits[7] ^ bits[9] ^ bits[10] ^ bits[14] ^ bits[15] ^ bits[16] ^ bits[17] ^ bits[18] ^ bits[21] ^ bits[22] ^ bits[24];
    p[6] = d30star ^ bits[3] ^ bits[5] ^ bits[6] ^ bits[8] ^ bits[9] ^ bits[10] ^ bits[11] ^ bits[13] ^ bits[15] ^ bits[19] ^ bits[22] ^ bits[23] ^ bits[24];

    // Final Comparison
    for (int i = 1; i <= 6; i++) {
        // Compare: p[i] != p_rx[i]
        if (p[i] != p_rx[i]) return false;
    }

    return true;
}

void NavDecoder::processTrackingMetrics(const CorrelatorResult& metrics) {
    // 1. Maintain internal tracking sample ledger timeline
    // This combines chunk offsets with your newly latched rollover timestamp
    if (metrics.rollover_sample_idx > 0) {
        _decoderSampleCounter = metrics.rollover_sample_idx;
    }

    // 2. Redirect vector arrays straight down into your original bits decoder engine block
    // Passing metrics.Pi and metrics.Pq provides prompt data tracking access
    if (!metrics.symbols.empty()) {
        processBits(metrics.symbols, metrics.Pi, metrics.Pq);
    }
}