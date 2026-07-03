#include "AcquisitionMgr.hpp"

std::vector<AcqResult> AcquisitionMgr::run(
    const RFE_Header_t &meta,
    RawSample *samples,
    size_t sample_count)
{
    std::vector<AcqResult> results;

    const size_t samples_per_ms =
        (size_t)(meta.fs_rate / 1000);

    const size_t needed_samples =
        samples_per_ms * NUM_MS;

    if (samples == nullptr || sample_count < needed_samples)
        return results;

    const uint32_t anchor =
        samples[0].sample_tick;

    const uint32_t aligned_anchor =
        anchor - (anchor % (uint32_t)samples_per_ms);

    std::vector<kiss_fft_cpx> aligned_data(
        FFT_SIZE * NUM_MS,
        {0, 0});

    for (int ms = 0; ms < NUM_MS; ++ms)
    {
        kiss_fft_cpx *dst =
            &aligned_data[ms * FFT_SIZE];

        RawSample *src =
            &samples[ms * samples_per_ms];

        for (size_t i = 0; i < samples_per_ms; ++i)
        {
            dst[i].r = (kiss_fft_scalar)src[i].i << ACQ_SAMPLE_SHIFT;
            dst[i].i = (kiss_fft_scalar)src[i].q << ACQ_SAMPLE_SHIFT;
        }

        // remaining FFT_SIZE - samples_per_ms stays zero-padded
    }

    const float samples_per_chip =
        (float)meta.fs_rate / 1023000.0f;

    for (int prn = 1; prn <= 32; ++prn)
    {
        AcqResult res =
            m_pcs.search(
                prn,
                aligned_data,
                ReceiverConfig::L1_IF_HZ,
                20,
                500.0f,
                aligned_anchor);

        if (res.snr > ACQ_SNR_THRESHOLD_DB)
        {
            res.codePhase =
                (float)res.peakIndex /
                samples_per_chip;

            results.push_back(res);
        }
    }

    std::vector<int> waas_prns = {131, 133, 135};

    for (int prn : waas_prns)
    {
        AcqResult res =
            m_pcs.search(
                prn,
                aligned_data,
                ReceiverConfig::L1_IF_HZ,
                20,
                500.0f,
                aligned_anchor);

        if (res.snr > ACQ_SNR_THRESHOLD_DB) // Acquisiton Threshold
        {
            res.codePhase =
                (float)res.peakIndex /
                samples_per_chip;

            results.push_back(res);
        }
    }

    return results;
}

AcqResult AcquisitionMgr::runSingle(
    const RFE_Header_t &meta,
    RawSample *samples,
    size_t sample_count,
    int prn,
    float centerFreq,
    int binRange,
    float binWidth)
{
    AcqResult empty = {};
    empty.prn = prn;
    empty.snr = 0.0f;

    const size_t samples_per_ms =
        (size_t)(meta.fs_rate / 1000);

    const size_t needed_samples =
        samples_per_ms * NUM_MS;

    if (samples == nullptr || sample_count < needed_samples)
        return empty;

    const uint32_t anchor =
        samples[0].sample_tick;

    const uint32_t aligned_anchor =
        anchor - (anchor % (uint32_t)samples_per_ms);

    std::vector<kiss_fft_cpx> aligned_data(
        FFT_SIZE * NUM_MS,
        {0, 0});

    for (int ms = 0; ms < NUM_MS; ++ms)
    {
        kiss_fft_cpx *dst =
            &aligned_data[ms * FFT_SIZE];

        RawSample *src =
            &samples[ms * samples_per_ms];

        for (size_t i = 0; i < samples_per_ms; ++i)
        {
            dst[i].r = (kiss_fft_scalar)src[i].i << ACQ_SAMPLE_SHIFT;
            dst[i].i = (kiss_fft_scalar)src[i].q << ACQ_SAMPLE_SHIFT;
        }
    }

    const float samples_per_chip =
        (float)meta.fs_rate / 1023000.0f;

    AcqResult res =
        m_pcs.search(
            prn,
            aligned_data,
            centerFreq,
            binRange,
            binWidth,
            aligned_anchor);

  if (binRange == 2 && binWidth == 50.0f)
{
    float carrierFreq =
        centerFreq + res.bin * binWidth;

    m_pcs.dumpLocalCorrelation(
        prn,
        aligned_data,
        carrierFreq,
        res.peakIndex,
        64,
        "pcs_local_refine.csv");
}  

    res.codePhase =
        (float)res.peakIndex /
        samples_per_chip;

    return res;
}

AcqResult AcquisitionMgr::runSingle(
    const RFE_Header_t &meta,
    RawSample *samples,
    size_t sample_count,
    int prn)
{
    return runSingle(
        meta,
        samples,
        sample_count,
        prn,
        ReceiverConfig::L1_IF_HZ,
        20,
        500.0f);
}