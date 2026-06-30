#include "TrackingEngine.h"
#include "HandoffRefiner.hpp"
#include <cmath>
// #define TRK_LAG_DEBUG
// #define DBG_NAV20

ChannelState::ChannelState(int p, double fs, const AcqResult &res, G2INIT s)
    : prn(p), result(res), sv(s)
{
  processor = std::make_unique<ChannelProcessor>(fs, result, sv);
  for (int p = 0; p < 20; ++p)
  {
    decoder[p] = std::make_unique<NavDecoder>(prn);
  }
}

void TrackingEngine::resetNavAccumulation(ChannelState &state)
{
  state.last_logged_sample_index = 0;
  state.nav20_sum = 0;
  state.nav20_count = 0;
  state.nav20_groups = 0;
  state.epochSymbols.clear();

  for (int p = 0; p < 20; ++p)
  {
    state.nav_phase_sum[p] = 0;
    state.nav_phase_score[p] = 0;
    state.nav_phase_windows[p] = 0;
    state.nav_phase_prompt_sum[p] = 0;
    state.nav_phase_prompt_score[p] = 0;
    state.nav_phase_prev_bit[p] = 0;
    state.nav_phase_has_prev_bit[p] = false;
    state.nav_phase_flip_count[p] = 0;
  }

  state.epoch_counter = 0;
}

bool TrackingEngine::beginTracking(
    ElasticReceiver &rx,
    const RFE_Header_t &meta,
    const AcqResult &pcs_acq,
    uint64_t acq_cursor,
    size_t acq_samples)
{
  RawSample *acq_ptr = nullptr;
  std::vector<RawSample> acq_window;

  if (!rx.get_window(
          acq_cursor,
          acq_ptr,
          (unsigned int)acq_samples,
          acq_window))
  {
    printf("[HANDOFF] failed to reload acquisition window\n");
    return false;
  }

  AcqResult track_input = pcs_acq;
  track_input.codePhase =
      pcsToTrackerCodePhase(pcs_acq.codePhase);

  AcqResult refined =
      refineHandoffWithTracker(
          (double)meta.fs_rate,
          rx.input_is_complex(),
          acq_ptr,
          acq_samples,
          track_input);

  activeChannels.clear();

  G2INIT sv(refined.prn, 0);

  activeChannels.emplace_back(
      refined.prn,
      (double)meta.fs_rate,
      refined,
      sv);

  auto &state = activeChannels.back();

  state.processor->setInputIsComplex(rx.input_is_complex());

  for (int phase = 0; phase < 20; ++phase)
  {
    state.decoder[phase]->setFocus(false);
  }

  state.total_tracked_ms = 0;
  state.handover_sample_tick =
      (uint32_t)std::round(refined.codePhase);
  state.handover_unix_time = acq_ptr[0].unix_time;
  state.sampleCursor = acq_cursor + acq_samples;

  resetNavAccumulation(state);

  printf("[TRK BEGIN] PRN=%d pcs=%.4f mapped=%.4f refined=%.4f cursor=%llu\n",
         pcs_acq.prn,
         pcs_acq.codePhase,
         track_input.codePhase,
         refined.codePhase,
         (unsigned long long)state.sampleCursor);

  return true;
}

bool TrackingEngine::captureReplayPackage(
    ElasticReceiver &rx,
    const RFE_Header_t &meta,
    const AcqResult &fresh,
    uint64_t fresh_cursor,
    size_t ms_samples,
    size_t capture_ms,
    bool input_is_complex,
    const char *basename)
{
  const size_t sample_count =
      capture_ms * ms_samples;

  RawSample *ptr = nullptr;
  std::vector<RawSample> window;

  if (!rx.get_window(
          fresh_cursor,
          ptr,
          (unsigned int)sample_count,
          window))
  {
    return false;
  }

  char raw_name[256];
  snprintf(raw_name,
           sizeof(raw_name),
           "%s.rawsamples",
           basename);

  FILE *fp = fopen(raw_name, "wb");
  if (!fp)
    return false;

  fwrite(window.data(),
         sizeof(RawSample),
         window.size(),
         fp);

  fclose(fp);

  char meta_name[256];
  snprintf(meta_name,
           sizeof(meta_name),
           "%s.meta",
           basename);

  FILE *mf = fopen(meta_name, "w");
  if (!mf)
    return false;

  fprintf(mf, "version=1\n");
  fprintf(mf, "fs_rate=%u\n", meta.fs_rate);
  fprintf(mf, "prn=%d\n", fresh.prn);
  fprintf(mf, "bin=%d\n", fresh.bin);
  fprintf(mf, "codePhase=%.8f\n", fresh.codePhase);
  fprintf(mf, "snr=%.3f\n", fresh.snr);
  fprintf(mf, "input_is_complex=%d\n",
          input_is_complex ? 1 : 0);
  fprintf(mf, "capture_ms=%zu\n", capture_ms);
  fprintf(mf, "sample_count=%zu\n", sample_count);
  fprintf(mf, "start_cursor=%llu\n",
          (unsigned long long)fresh_cursor);

  fclose(mf);

  printf("[CAPTURE] %zu samples -> %s\n",
         window.size(),
         raw_name);

  return true;
}

void TrackingEngine::processEpoch(
    ChannelState &state,
    const EpochResult &epoch,
    const RFE_Header_t &meta,
    FILE *out)
{
  int8_t sym = epoch.symbol;
  int32_t prompt_i = epoch.Pi;
  static constexpr uint64_t max_iq_log_rows = 1000;
  static bool iq_log_done = false;

  if (!iq_log_done && iq_log == nullptr)
  {
    iq_log = fopen("tracking_iq.csv", "w");
  }

  if (iq_log && !iq_log_header_written)
  {
    fprintf(iq_log,
            "epoch,prn,"
            "Ei,Eq,Pi,Pq,Li,Lq,"
            "symbol,snr,doppler,code_phase,"
            "E_mag,P_mag,L_mag,"
            "dll_disc,pll_disc\n");
    iq_log_header_written = true;
  }

  if (iq_log && iq_log_rows < max_iq_log_rows)
  {
    double E_mag =
        std::sqrt((double)epoch.Ei * epoch.Ei +
                  (double)epoch.Eq * epoch.Eq);

    double P_mag =
        std::sqrt((double)epoch.Pi * epoch.Pi +
                  (double)epoch.Pq * epoch.Pq);

    double L_mag =
        std::sqrt((double)epoch.Li * epoch.Li +
                  (double)epoch.Lq * epoch.Lq);

    double dll_disc = 0.0;
    double denom = E_mag + L_mag;

    if (denom > 1e-9)
    {
      dll_disc = (E_mag - L_mag) / denom;
    }

    double pll_disc = std::atan2((double)epoch.Pq, (double)epoch.Pi);

    fprintf(iq_log,
            "%llu,%d,"
            "%d,%d,%d,%d,%d,%d,"
            "%d,%.1f,%.1f,%.3f,"
            "%.1f,%.1f,%.1f,"
            "%.9f,%.9f\n",
            state.epoch_counter,
            state.prn,
            epoch.Ei,
            epoch.Eq,
            epoch.Pi,
            epoch.Pq,
            epoch.Li,
            epoch.Lq,
            sym,
            state.last_snr,
            state.last_doppler_hz,
            state.last_code_phase,
            E_mag,
            P_mag,
            L_mag,
            dll_disc,
            pll_disc);

    iq_log_rows++;

    if (iq_log_rows == max_iq_log_rows)
    {
      fflush(iq_log);
      fclose(iq_log);
      iq_log = nullptr;
      iq_log_done = true;

      printf("\n[IQLOG] Closed tracking_iq.csv after %llu rows\n",
             iq_log_rows);
    }
  }

  for (int phase = 0; phase < 20; ++phase)
  {
    state.nav_phase_sum[phase] += sym;
    state.nav_phase_prompt_sum[phase] += prompt_i;

    if (((state.epoch_counter + 1 + 20 - phase) % 20) == 0)
    {
      int64_t prompt_sum = state.nav_phase_prompt_sum[phase];

      int8_t phase_bit =
          (prompt_sum >= 0) ? 1 : -1;

      state.decoder[phase]->processBit(phase_bit);

      state.nav_phase_score[phase] +=
          std::abs(state.nav_phase_sum[phase]);

      state.nav_phase_prompt_score[phase] =
          (state.nav_phase_prompt_score[phase] * 31 +
           std::llabs(prompt_sum)) /
          32;

      state.nav_phase_windows[phase]++;

      state.nav_phase_sum[phase] = 0;
      state.nav_phase_prompt_sum[phase] = 0;
    }
  }

  state.epoch_counter++;

  if ((state.epoch_counter % 5000) == 0)
  {
    int best_phase = -1;
    int second_phase = -1;

    double best_avg = -1.0;
    double second_avg = -1.0;

    printf("\n[NAVPHASE]");

    for (int p = 0; p < 20; ++p)
    {
      double avg = 0.0;

      if (state.nav_phase_windows[p] > 0)
      {
        avg = (double)state.nav_phase_prompt_score[p];
      }

      printf(" %02d:%.1fk", p, avg / 1000.0);

      if (avg > best_avg)
      {
        second_avg = best_avg;
        second_phase = best_phase;

        best_avg = avg;
        best_phase = p;
      }
      else if (avg > second_avg)
      {
        second_avg = avg;
        second_phase = p;
      }
    }

    double ratio =
        (second_avg > 0.0)
            ? (best_avg / second_avg)
            : 0.0;

    state.nav_phase_best = best_phase;
    state.nav_phase_ratio = ratio;

    printf(
        "\n best=%02d %.1fk second=%02d %.1fk ratio=%.3f\n",
        best_phase,
        best_avg / 1000.0,
        second_phase,
        second_avg / 1000.0,
        ratio);

    printf("[NAVDEC]");
    int best_dec_phase = -1;
    int best_dec_score = -999999;

    int best_pre = 0;
    int best_pass = 0;
    int best_fail = 0;

    for (int p = 0; p < 20; ++p)
    {
      int pre = state.decoder[p]->getPreambleCandidateCount();
      int pass = state.decoder[p]->getParityPassCount();
      int fail = state.decoder[p]->getParityFailCount();

      int score = pass * 20 - fail;

      if (score > best_dec_score)
      {
        best_dec_score = score;
        best_dec_phase = p;

        best_pre = pre;
        best_pass = pass;
        best_fail = fail;
      }
    }

    printf(" D%02d p%d +%d -%d sc%d\n",
           best_dec_phase,
           best_pre,
           best_pass,
           best_fail,
           best_dec_score);
  }

  char epoch_symbol = (sym > 0) ? '#' : '-';

  state.epochSymbols.push_back(sym);
  state.nav20_sum += sym;
  state.nav20_count++;

  if (state.nav20_count == 20)
  {
    int8_t nav_bit = (state.nav20_sum >= 0) ? 1 : -1;
    state.nav20_groups++;
#ifdef DBG_NAV20
    if ((state.nav20_groups % 20) == 0)
    {
      printf("\n[NAV20] PRN %d bit=%c sum=%d\n",
             state.prn,
             nav_bit > 0 ? '#' : '-',
             state.nav20_sum);
    }
#endif
    state.nav20_sum = 0;
    state.nav20_count = 0;
  }

  uint64_t stable_fs_rate = (uint64_t)meta.fs_rate;
  uint64_t sub_second_ticks = epoch.sample_tick % stable_fs_rate;
  uint32_t calculated_ms =
      (uint32_t)((sub_second_ticks * 1000) / stable_fs_rate);

  uint32_t unixSecond = epoch.unix_time;
  uint32_t msCount = calculated_ms;

  if (file_logging_enabled && logged_ms < max_logged_ms)
  {
    fprintf(
        out,
        "\n%s PRN=%d epoch_tick=%u ms=%u epoch_idx=%llu off=%d epochN=%u epochPi=%d epochPq=%d epochSym=%c\n",
        get_iso8601_timestamp(unixSecond, msCount).c_str(),
        state.prn,
        epoch.sample_tick,
        calculated_ms,
        epoch.sample_index,
        epoch.offset_samples,
        epoch.sample_count,
        epoch.Pi,
        epoch.Pq,
        epoch_symbol);

    logged_ms++;

    if (logged_ms == max_logged_ms)
    {
      fflush(out);
      file_logging_enabled = false;
      printf("\n[LOG] Stopped file logging after %llu ms\n", logged_ms);
    }
  }
}

bool TrackingEngine::step(
    ElasticReceiver &rx,
    const RFE_Header_t &meta,
    uint32_t focusPRN,
    FILE *out,
    bool &acq_needed)
{
  const size_t ms_samples = (size_t)(meta.fs_rate / 1000.0);
  const int feed_samples = (unsigned int)ms_samples; // Process 1 ms of data at a time
  bool did_work = false;

  for (auto &state : activeChannels)
  {
    if (state.prn != (int)focusPRN)
      continue;

    while (true)
    {
      uint64_t write = rx.get_write_index();

      auto timing =
          rx.get_timing_status(state.sampleCursor, ms_samples);

      uint64_t oldest_available = timing.oldest_available;

      int64_t lag_samples =
          (int64_t)write - (int64_t)state.sampleCursor;

      int64_t margin_samples =
          (int64_t)state.sampleCursor - (int64_t)oldest_available;

      double lag_ms =
          (double)lag_samples / (double)ms_samples;

      double margin_ms =
          (double)margin_samples / (double)ms_samples;

      if (state.sampleCursor < oldest_available)
      {
        uint64_t aligned_write =
            write - (write % ms_samples);

        uint64_t new_cursor =
            aligned_write - ms_samples;

        printf(
            "\n[TRK JUMP] PRN %d lag=%.1f margin=%.1f cursor=%llu oldest=%llu write=%llu -> %llu\n",
            state.prn,
            lag_ms,
            margin_ms,
            state.sampleCursor,
            oldest_available,
            write,
            new_cursor);

        state.sampleCursor = new_cursor;

        resetNavAccumulation(state);

        continue;
      }

      if (write < state.sampleCursor + feed_samples)
        break;

      RawSample *ms_ptr = nullptr;
      std::vector<RawSample> ms_window;

      if (!rx.get_window(
              state.sampleCursor,
              ms_ptr,
              feed_samples,
              ms_window))
      {
        break;
      }

      uint64_t current_cursor = state.sampleCursor;
      uint64_t this_index = ms_ptr[0].sample_index;

      if (this_index != current_cursor)
      {
        printf(
            "\n[CURSOR MISMATCH] cursor=%llu sample_index=%llu\n",
            current_cursor,
            this_index);
      }

      if (state.last_logged_sample_index != 0)
      {
        uint64_t expected = state.last_logged_sample_index + feed_samples;
        if (this_index != expected)
        {
          int64_t delta = (int64_t)this_index - (int64_t)expected;
          printf("\n[CURSOR MISMATCH] cursor=%llu sample_index=%llu delta=%lld\n",
                 current_cursor, this_index, delta);
        }
      }

      state.last_logged_sample_index = this_index;

      state.processor->setLoopEnables(true, true);
      CorrelatorResult res = state.processor->Correlator(ms_ptr, feed_samples);

      did_work = true;

      if (res.consumed_sample_count != feed_samples)
      {
        printf(
            "\n[TRK NOTE] correlator reported consumed=%zu, forcing feed=%d\n",
            res.consumed_sample_count,
            feed_samples);
      }

      state.sampleCursor += feed_samples;

      state.total_tracked_ms++;

if (state.total_tracked_ms > 500 &&
    res.snr < 6.0f &&
    std::abs(res.Pi) < 5000 &&
    std::abs(res.Pq) > 5000)
{
    printf("\n[LOCK LOST] PRN %d reacquiring\n", state.prn);
    activeChannels.clear();
    acq_needed = true;
    return did_work;
}

      state.last_snr = res.snr;
      state.last_doppler_hz = res.doppler_hz;
      state.last_code_phase = res.code_phase;

      for (const auto &epoch : res.epochs)
      {
        processEpoch(state, epoch, meta, out);
      }

      if (state.total_tracked_ms % 100 == 0)
      {
        //          printf(
        //              "epoch[%zu] sym=%d Pi=%d Pq=%d N=%u off=%d\n", idx, epoch.symbol,
        //              epoch.Pi, epoch.Pq, epoch.sample_count, epoch.offset_samples);
        int pi_k = res.Pi / 1000;
        int pq_k = res.Pq / 1000;
        printf(
            "\r[TRK] %03d S%5.1f C%7.2f D%+7.1f Pi%+5dk Pq%+4dk N%02d R%4.2f",
            state.prn,
            res.snr,
            res.code_phase,
            res.doppler_hz,
            pi_k,
            pq_k,
            state.nav_phase_best,
            state.nav_phase_ratio);

        fflush(stdout);

#ifdef TRK_LAG_DEBUG
        printf(
            "\n[LAG] PRN %d lag=%.1f ms cursor=%llu oldest=%llu write=%llu\n",
            state.prn,
            lag_ms,
            state.sampleCursor,
            oldest_available,
            write);
#endif
      }
    }
  }
  return did_work;
}