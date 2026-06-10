#include "TrackingEngine.h"

ChannelState::ChannelState(int p, double fs, const AcqResult &res, G2INIT s)
    : prn(p), result(res), sv(s)
{
  processor = std::make_unique<ChannelProcessor>(fs, result, sv);
  decoder = std::make_unique<NavDecoder>(p);
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
            "[TRK JUMP] PRN %d lag=%.1f margin=%.1f cursor=%llu oldest=%llu write=%llu -> %llu\n",
            state.prn,
            lag_ms,
            margin_ms,
            state.sampleCursor,
            oldest_available,
            write,
            new_cursor);

        state.sampleCursor = new_cursor;
        state.last_logged_sample_index = 0;
        state.nav20_sum = 0;
        state.nav20_count = 0;
        state.epochSymbols.clear();

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
            "[CURSOR MISMATCH] cursor=%llu sample_index=%llu\n",
            current_cursor,
            this_index);
      }

      if (state.last_logged_sample_index != 0)
      {
        uint64_t expected = state.last_logged_sample_index + feed_samples;
        if (this_index != current_cursor)
        {
          int64_t delta = (int64_t)this_index - (int64_t)expected;
          printf("[CURSOR MISMATCH] cursor=%llu sample_index=%llu delta=%lld\n",
                 current_cursor, this_index, delta);
        }
      }

      state.last_logged_sample_index = this_index;

      CorrelatorResult res = state.processor->Correlator(ms_ptr, feed_samples);

      did_work = true;

      if (res.consumed_sample_count != feed_samples)
      {
        printf(
            "[TRK NOTE] correlator reported consumed=%zu, forcing feed=%d\n",
            res.consumed_sample_count,
            feed_samples);
      }

      state.sampleCursor += feed_samples;

      state.total_tracked_ms++;

      for (size_t idx = 0; idx < res.epochs.size(); ++idx)
      {
        const auto &epoch = res.epochs[idx];

        int8_t sym = epoch.symbol;
        for (int phase = 0; phase < 20; ++phase)
        {
          state.nav_phase_sum[phase] += sym;

          if (((state.epoch_counter + 1 + 20 - phase) % 20) == 0)
          {
            state.nav_phase_score[phase] += std::abs(state.nav_phase_sum[phase]);
            state.nav_phase_windows[phase]++;
            state.nav_phase_sum[phase] = 0;
          }
        }

        state.epoch_counter++;

        if ((state.epoch_counter % 400) == 0)
        {
          int best_phase = 0;
          int best_score = state.nav_phase_score[0];
          if ((state.epoch_counter % 400) == 0)
          {
            int best_phase = 0;
            double best_avg = 0.0;

            printf("[NAVPHASE]");

            for (int p = 0; p < 20; ++p)
            {
              double avg = 0.0;

              if (state.nav_phase_windows[p] > 0)
              {
                avg =
                    (double)state.nav_phase_score[p] /
                    (double)state.nav_phase_windows[p];
              }

              printf(" %02d:%.2f", p, avg);

              if (avg > best_avg)
              {
                best_avg = avg;
                best_phase = p;
              }
            }

            printf(" best=%02d avg=%.2f\n", best_phase, best_avg);
          }
        }
        char epoch_symbol = (sym > 0) ? '#' : '-';

        state.epochSymbols.push_back(sym);
        state.nav20_sum += sym;
        state.nav20_count++;

        if (state.nav20_count == 20)
        {
          int8_t nav_bit = (state.nav20_sum >= 0) ? 1 : -1;

          state.nav20_groups++;

          if ((state.nav20_groups % 20) == 0)
          {
            printf("[NAV20] PRN %d bit=%c sum=%d\n",
                   state.prn,
                   nav_bit > 0 ? '#' : '-',
                   state.nav20_sum);
          }

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
              "%s PRN=%d epoch_tick=%u ms=%u epoch_idx=%llu off=%d epochN=%u epochPi=%d epochPq=%d epochSym=%c\n",
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
            printf("[LOG] Stopped file logging after %llu ms\n", logged_ms);
          }
        }
      }
      if (state.total_tracked_ms % 100 == 0)
      {
        //          printf(
        //              "epoch[%zu] sym=%d Pi=%d Pq=%d N=%u off=%d\n", idx, epoch.symbol,
        //              epoch.Pi, epoch.Pq, epoch.sample_count, epoch.offset_samples);
        printf(
            "[TRK] PRN %3d SNR:%5.1f dF:%8.1f Code:%7.2f used:%5zu blockPi:% 7d blockPq:% 7d epochs:%zu\n",
            state.prn,
            res.snr,
            res.doppler_hz,
            res.code_phase,
            res.consumed_sample_count,
            res.Pi,
            res.Pq,
            res.epochs.size());

        printf(
            "[LAG] PRN %d lag=%.1f ms cursor=%llu oldest=%llu write=%llu\n",
            state.prn,
            lag_ms,
            state.sampleCursor,
            oldest_available,
            write);
      }
    }
  }
  return did_work;
}