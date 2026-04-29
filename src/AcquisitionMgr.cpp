#include "AcquisitionMgr.hpp"

std::vector<AcqResult> AcquisitionMgr::run(const RFE_Header_t& meta, uint8_t* raw_ptr) {
    std::vector<AcqResult> results;
    
    // Telemetry is now anchored: meta.sample_tick is the START of raw_ptr
    const uint32_t anchor = meta.sample_tick;
    const uint32_t aligned_anchor = anchor - (anchor % 16368);

    // 1. Unpack and Align 5ms of data
    // raw_ptr is now guaranteed to be 1ms-aligned by ElasticReceiver
    std::vector<kiss_fft_cpx> aligned_data(FFT_SIZE * NUM_MS, {0, 0});
    for (int ms = 0; ms < NUM_MS; ms++) {
        uint8_t *ms_source = raw_ptr + (ms * 8184); 
        kiss_fft_cpx *ms_dest = &aligned_data[ms * FFT_SIZE];
        for (size_t i = 0; i < 8184; ++i) {
            unpackL1IF(ms_source[i], ms_dest[2 * i], ms_dest[2 * i + 1 ]);
        }
    }

    // 2. Search GPS PRNs (1-32)
    for (int prn = 1; prn <= 32; prn++) {
        AcqResult res = m_pcs.search(prn, aligned_data, 4.092e6f, 20, 500.0f, aligned_anchor);
        if (res.snr > 9.0) {
            // Since the buffer is epoch-aligned, codePhase is just peakIndex in chips
            res.codePhase = (float)(res.peakIndex / 16.0f); 
            results.push_back(res);
        }
    }

    // 3. Search WAAS PRNs
    std::vector<int> waas_prns = {131, 133, 135};
    for (int prn : waas_prns) {
        AcqResult w_res = m_pcs.search(prn, aligned_data, 4.092e6f, 20, 500.0f, aligned_anchor);
        if (w_res.snr > 9.0) {
            w_res.codePhase = (float)(w_res.peakIndex / 16.0f);
            results.push_back(w_res);
        }
    }

    return results;
}
