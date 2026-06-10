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
  bool did_work = false;

  for (auto &state : activeChannels)
  {
    if (state.prn != (int)focusPRN)
      continue;

    uint64_t write = rx.get_write_index();
    uint64_t ring_capacity = ms_samples * 250;

    if (state.sampleCursor + ms_samples < write - ring_capacity)
    {
      printf(
          "[TRK STALE] PRN %d cursor=%llu write=%llu capacity=%llu -- reacquire\n",
          state.prn,
          state.sampleCursor,
          write,
          ring_capacity);

      acq_needed = true;
      activeChannels.clear();
      break;
    }

    while (rx.get_write_index() >= state.sampleCursor + ms_samples)
    {
      RawSample *ms_ptr = nullptr;
      std::vector<RawSample> ms_window;

      if (!rx.get_window(
              state.sampleCursor,
              ms_ptr,
              (unsigned int)ms_samples,
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
        uint64_t expected = state.last_logged_sample_index + ms_samples;

        if (this_index != expected)
        {
          printf(
              "[TRACK GAP] expected=%llu got=%llu delta=%lld ms_delta=%.3f\n",
              expected,
              this_index,
              (long long)(this_index - expected),
              (double)(this_index - expected) / (double)ms_samples);
        }
      }

      state.last_logged_sample_index = this_index;

      CorrelatorResult res =
          state.processor->Correlator(
              ms_ptr,
              (unsigned int)ms_samples);

      state.sampleCursor += ms_samples;
      state.total_tracked_ms++;

      // ---- Existing code phase update ----

      double activeCodeFreq =
          1023000.0 +
          (double)res.doppler_hz / 1540.0;

      if (res.epoch_offset_samples != -1)
      {
        double dynamic_samples_per_chip =
            (double)(ms_samples * 1000.0) /
            activeCodeFreq;

        double chips_to_end =
            (double)res.epoch_offset_samples /
            dynamic_samples_per_chip;

        state.result.codePhase =
            std::fmod(
                1023.0 - chips_to_end,
                1023.0);
      }
      else
      {
        double chips_advanced =
            (double)ms_samples *
            activeCodeFreq /
            (double)(meta.fs_rate);

        state.result.codePhase =
            std::fmod(
                state.result.codePhase +
                    chips_advanced,
                1023.0);
      }

      // ---- Timing comes directly from ring ----

      if (res.epoch_valid)
      {
        const RawSample &sample = ms_ptr[0];

        uint64_t stable_fs_rate = (uint64_t)meta.fs_rate;
        uint64_t sub_second_ticks = sample.sample_tick % stable_fs_rate;
        uint32_t calculated_ms =
            (uint32_t)((sub_second_ticks * 1000) / stable_fs_rate);

        uint32_t unixSecond = sample.unix_time;
        uint32_t msCount = calculated_ms;

        // 1 symbol per 1 ms integration
        const char symbol = (res.Pi > 0) ? '#' : '-';
        int8_t sym = (res.Pi >= 0) ? 1 : -1;

        state.epochSymbols.push_back(sym);
        state.nav20_sum += sym;
        state.nav20_count++;

        /*
        if (state.nav20_count == 20)
        {
          int8_t nav_bit = (state.nav20_sum >= 0) ? 1 : -1;

          printf("[NAV BIT] PRN %d bit=%d sum=%d\n",
                 state.prn,
                 nav_bit,
                 state.nav20_sum);

          state.nav20_sum = 0;
          state.nav20_count = 0;
        } */
        if (file_logging_enabled && logged_ms < max_logged_ms)
        {
          fprintf(
              out,
              "%s ",
              get_iso8601_timestamp(unixSecond, msCount).c_str());

          printCorrelatorData(out, res);

          fprintf(
              out,
              " tick=%u ms=%u idx=%llu ",
              sample.sample_tick,
              calculated_ms,
              sample.sample_index);
          fprintf(out, " idx=%llu ", sample.sample_index);
          fprintf(out, " | Bits: %c\n", symbol);

          logged_ms++;

          if (logged_ms == max_logged_ms)
          {
            fflush(out);
            file_logging_enabled = false;
            printf("[LOG] Stopped file logging after %llu ms\n", logged_ms);
          }
        }

        // Lightweight console sanity check
        if (state.total_tracked_ms % 100 == 0)
        {
          printf(
              "[TRK] PRN %3d SNR:%5.1f dF:%8.1f Code:%7.2f Pi:% 7d Pq:% 7d Bit:%c\n",
              state.prn,
              res.snr,
              res.doppler_hz,
              res.code_phase,
              res.Pi,
              res.Pq,
              symbol);
        }
      }
    }
  }
  return did_work;
};