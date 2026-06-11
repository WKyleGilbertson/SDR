#include "TrackingEngine.h"
// #define TRK_LAG_DEBUG
// #define DBG_NAV20

ChannelState::ChannelState(int p, double fs, const AcqResult &res, G2INIT s)
    : prn(p), result(res), sv(s)
{
  processor = std::make_unique<ChannelProcessor>(fs, result, sv);
  decoder = std::make_unique<NavDecoder>(p);
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

void TrackingEngine::processEpoch(
    ChannelState &state,
    const EpochResult &epoch,
    const RFE_Header_t &meta,
    FILE *out)
{
  int8_t sym = epoch.symbol;
  int32_t prompt_i = epoch.Pi;

  for (int phase = 0; phase < 20; ++phase)
  {
    state.nav_phase_sum[phase] += sym;
    state.nav_phase_prompt_sum[phase] += prompt_i;

    if (((state.epoch_counter + 1 + 20 - phase) % 20) == 0)
    {
      state.nav_phase_score[phase] +=
          std::abs(state.nav_phase_sum[phase]);

      state.nav_phase_prompt_score[phase] =
          (state.nav_phase_prompt_score[phase] * 31 +
           std::llabs(state.nav_phase_prompt_sum[phase])) /
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
      uint64_t ring_capacity = ms_samples * 250;

      uint64_t oldest_available =
          (write > ring_capacity)
              ? (write - ring_capacity)
              : 0;

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