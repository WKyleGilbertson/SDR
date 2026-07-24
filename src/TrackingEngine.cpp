#include "TrackingEngine.h"
#include "HandoffRefiner.hpp"
#include <cmath>
// #define TRK_LAG_DEBUG

ChannelState::ChannelState(int p, double fs, const AcqResult &res, G2INIT s)
    : prn(p), result(res), sv(s)
{
  processor = std::make_unique<ChannelProcessor>(fs, result, sv);
  decoder = std::make_unique<NavDecoder>(prn, fs);
}

void TrackingEngine::resetNavAccumulation(ChannelState &state)
{
  state.last_logged_sample_index = 0;
  state.epochSymbols.clear();

  state.decoder->setFocus(false);
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

  activeChannels.remove_if(
      [&](const ChannelState &ch)
      {
        return ch.prn == refined.prn;
      });

  G2INIT sv(refined.prn, 0);

  activeChannels.emplace_back(
      refined.prn,
      (double)meta.fs_rate,
      refined,
      sv);

  auto &state = activeChannels.back();

  state.processor->setInputIsComplex(rx.input_is_complex());

  if (state.decoder)
  {
    state.decoder->setFocus(false);
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
            "symbol,snr,doppler,carrier_nco_hz,code_phase,"
            "E_mag,P_mag,L_mag,"
            "dll_disc,pll_disc,is_locked\n");
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
            "%d,%.1f,%.1f,%.1f,%.3f,"
            "%.1f,%.1f,%.1f,"
            "%.9f,%.9f,%d\n",
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
            state.last_carrier_nco_hz,
            state.last_code_phase,
            E_mag,
            P_mag,
            L_mag,
            dll_disc,
            pll_disc,
            state.last_is_locked ? 1 : 0);

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

  state.epoch_counter++;

  char epoch_symbol = (sym > 0) ? '#' : '-';

  state.epochSymbols.push_back(sym);

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

  for (auto it = activeChannels.begin(); it != activeChannels.end();)
  {
    ChannelState &state = *it;

    // 1. Explicitly evaluate focus based on the requested PRN
    bool isCurrentChannelFocused = (state.prn == (int)focusPRN);

    // 2. Broadcast the focus state to ALL 20 parallel decoders
    if (state.decoder)
    {
      state.decoder->setFocus(isCurrentChannelFocused);
    }

    if (!isCurrentChannelFocused)
    {
      ++it;
      continue;
    }

    // Keep processing the rest of your focus-specific tracking logic below...
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

      double pMag = std::hypot((double)res.Pi, (double)res.Pq);

      bool badEpoch = ((res.snr < 6.0f) && (pMag < 8000));

      if (badEpoch)
        state.badLockEpochs++;
      else
        state.badLockEpochs = 0;

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

      // ==========================================================
      // DYNAMIC LOOP STATE MACHINE: FLL Pull-in -> PLL -> Fallback
      // ==========================================================
      if (state.total_tracked_ms == 1)
      {
        state.processor->setLoopMode(LoopMode::Acquisition);
        state.processor->setUseFLL(true); // Aggressive frequency pull-in
      }
      else if (state.total_tracked_ms == 200)
      {
        state.processor->setLoopMode(LoopMode::PullIn);
        state.processor->setUseFLL(true); // Narrowing FLL
      }
      else if (state.total_tracked_ms == 800)
      {
        state.processor->setLoopMode(LoopMode::Tracking);
        state.processor->setUseFLL(false); // Transition to exact PLL for decoding
      }

      // 1. ABSOLUTE DEATH CHECK FIRST
      // If we spend 50ms in the noise floor, the code phase is gone. Kill it.
      if (state.badLockEpochs >= 50)
      {
        printf("\n[LOCK LOST] PRN %d queued for focused reacquire\n", state.prn);
        queueReacquire((uint32_t)state.prn);
        it = activeChannels.erase(it);
        acq_needed = true;
        return did_work;
      }
      // 2. FALLBACK CHECK
      // If we just have minor jitter, try dropping back to FLL.
      else if (state.total_tracked_ms > 800 && state.badLockEpochs >= 5)
      {
        printf("\n[FALLBACK] PRN %d phase jitter detected. Falling back to FLL.\n", state.prn);
        state.processor->setLoopMode(LoopMode::PullIn);
        state.processor->setUseFLL(true);
        state.total_tracked_ms = 200; // Reset tracking timeline to ride out the FLL stage again
        state.badLockEpochs = 0; // Reset the bad lock counter to avoid immediate reacquire
      }
      // ==========================================================

      state.last_snr = res.snr;
      state.last_doppler_hz = res.doppler_hz;
      state.last_code_phase = res.code_phase;

      state.last_carrier_nco_hz = res.carrier_nco_hz;
      state.last_is_locked = res.is_locked;
      // ==========================================================================
      // INJECT HIGH-RES SLIDING PREAMBLE SEARCH ENGINE (WITH ACCELERATED FEED)
      // ==========================================================================
      // Make a mutable copy of the result to ensure lock conditions and
      // symbol segments are perfectly formed for the sliding matching logic.
      CorrelatorResult master_res = res;

      // Force the lock state to true so the sliding register accumulates
      // history even while the tracking loop filters are pulling in.
      master_res.is_locked = true;

      // Ensure the master result has at least the current 1ms epoch symbols populated
      if (master_res.epochs.empty() && state.total_tracked_ms > 0)
      {
        // Fallback boundary safety: pull directly from the master prompt inversion
        EpochResult dummy_epoch;
        dummy_epoch.symbol = (res.Pi >= 0) ? 1 : -1;
        dummy_epoch.Pi = res.Pi;
        dummy_epoch.Pq = res.Pq;
        master_res.epochs.push_back(dummy_epoch);
        master_res.numSymbols = 1;
      }

      // Route the fortified continuous metrics through our master stream decoder
      // Route the fortified continuous metrics through our master stream decoder
      if (state.decoder)
      {
        if (isCurrentChannelFocused)
        {
          if (master_res.epochs.empty())
          {
            printf("[WARNING-TE] master_res.epochs is empty! NavDecoder will starve.\n");
          }
          // else {
          //     printf("[DEBUG-TE] Handing off %zu epochs to NavDecoder for PRN %d\n",
          //            master_res.epochs.size(), state.prn);
          // }
        }
        state.decoder->processTrackingMetrics(master_res);
      }

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
        printf("\r[TE]PRN %3d | SNR %5.1f | dF %7.1f | Code %8.3f | I %3dk | Q %3dk ",
               //"N%02d R%4.2f",
               res.prn, res.snr, res.doppler_hz, res.code_phase, pi_k, pq_k);
        //  state.nav_phase_best, state.nav_phase_ratio);

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
    } // End while
    ++it;
  } // end for
  return did_work;
} // end step()

double TrackingEngine::getExactTransmitTime(int prn)
{
    // Loop through your list of active tracking channels
    for (auto& chan : activeChannels) 
    {
        // Find the channel tracking our requested PRN
        if (chan.prn == prn) 
        {
            // chan.decoder is a unique_ptr, so we use '->'
            // We pass in the most recently saved code phase for this channel
            if (chan.decoder) {
                return chan.decoder->getExactTransmitTime(chan.last_code_phase);
            }
        }
    }
    return 0.0; // Return 0 if the channel isn't locked/tracking
}