#include "AcquisitionMgr.hpp"

std::vector<AcqResult> AcquisitionMgr::run(
    const RFE_Header_t& meta,
    RawSample* samples,
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
        kiss_fft_cpx* dst =
            &aligned_data[ms * FFT_SIZE];

        RawSample* src =
            &samples[ms * samples_per_ms];

        for (size_t i = 0; i < samples_per_ms; ++i)
        {
            dst[i].r = (kiss_fft_scalar)src[i].i;
            dst[i].i = (kiss_fft_scalar)src[i].q;
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
                4.092e6f,
                20,
                500.0f,
                aligned_anchor);

        if (res.snr > 9.0f)
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
                4.092e6f,
                20,
                500.0f,
                aligned_anchor);

        if (res.snr > 9.0f)
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
    int prn)
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
            dst[i].r = (kiss_fft_scalar)src[i].i;
            dst[i].i = (kiss_fft_scalar)src[i].q;
        }
    }

    const float samples_per_chip =
        (float)meta.fs_rate / 1023000.0f;

    AcqResult res =
        m_pcs.search(
            prn,
            aligned_data,
            4.092e6f,
            20,
            500.0f,
            aligned_anchor);

    res.codePhase =
        (float)res.peakIndex /
        samples_per_chip;

    return res;
}