#include "ChannelProcessor.h"

ChannelProcessor::ChannelProcessor(double fs_rate) 
    : _fs(fs_rate), _nco(10, (float)fs_rate) {
}

CorrRes ChannelProcessor::process(const uint8_t* data, size_t count, double freq) {
    float acc_i = 0.0f;
    float acc_q = 0.0f;
    
    _nco.SetFrequency((float)freq);

    for (size_t i = 0; i < count; ++i) {
        uint8_t b = data[i];
        
        // Two samples packed per byte
        for (int s = 0; s < 2; s++) {
            uint32_t idx = _nco.clk(); 
            float s_val = _nco.sine(idx);
            float c_val = _nco.cosine(idx);

            // Bit unpacking logic:
            // s=0: bit 0 (Sign), bit 4 (Mag)
            // s=1: bit 2 (Sign), bit 6 (Mag)
            int sign_bit = (b >> (s * 2)) & 1;
            int mag_bit  = (b >> (s * 2 + 4)) & 1;
            
            double val = _map(sign_bit, mag_bit);

            // Simple Real-to-Complex Mix
            acc_i += (float)(val * c_val);
            acc_q -= (float)(val * s_val); 
        }
    }
    return { (double)acc_i, (double)acc_q };
}