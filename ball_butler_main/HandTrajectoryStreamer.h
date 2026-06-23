// HandTrajectoryStreamer.h
#pragma once
#include "BallButlerConfig.h"
#include "CanInterface.h"
#include "TrajFrame.h"
#include "YawAxis.h"   // Layer C: read yaw err/rate at fire time
#include <math.h>      // fabsf

struct HandTrajectoryStreamer {
  // ODrive enums (axis state constants live in BallButlerConfig.h)
  static constexpr uint32_t CONTROL_MODE_POSITION  = 3u;
  static constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1u;

  CanInterface& can;
  const TrajFrame* frames = nullptr;
  size_t n = 0, idx = 0;
  uint32_t node = 0;
  bool active = false;
  uint64_t t_offset_us = 0;  // absolute wall-clock offset in microseconds

  // Layer C — fire-time settled confirm (set per-arm; checked once before frame 0).
  bool      require_settled_ = false;
  YawAxis*  yaw_ = nullptr;
  uint32_t  pitch_node_ = 0;
  float     yaw_err_tol_ = 0.0f;
  float     yaw_rate_tol_ = 0.0f;
  bool      aborted_ = false;  // last arm aborted by the settle gate (no frames sent)

  explicit HandTrajectoryStreamer(CanInterface& c) : can(c) {}

  // time_offset_us: absolute wall-clock reference in microseconds.
  //   frames[i] is sent when wallTimeUs() >= time_offset_us + frames[i].t_s * 1e6.
  //   • For "start now": pass can.wallTimeUs()
  //   • For "decel at absolute time T": pass T (in µs)
  // Layer C (throws only): pass require_settled=true with the yaw axis, pitch node
  //   id and tolerances; the streamer confirms the platform is settled before it
  //   sends the first (wind-up) frame, and aborts cleanly if not.
  bool arm(uint32_t node_id, const TrajFrame* f, size_t count, uint64_t time_offset_us = 0,
           bool require_settled = false, YawAxis* yaw = nullptr, uint32_t pitch_node = 0,
           float yaw_err_tol_deg = 0.0f, float yaw_rate_tol_dps = 0.0f) {
    if (!f || count == 0) return false;
    node = node_id; frames = f; n = count; idx = 0; this->t_offset_us = time_offset_us;
    require_settled_ = require_settled; yaw_ = yaw; pitch_node_ = pitch_node;
    yaw_err_tol_ = yaw_err_tol_deg; yaw_rate_tol_ = yaw_rate_tol_dps;
    aborted_ = false;

    if (!can.isAxisHomed(node_id)) {
      if (Serial) Serial.printf("[Gate] Trajectory arm rejected for node %lu (not homed)\n", (unsigned long)node_id);
      return false;
    }

    // 1) CLOSED_LOOP + POSITION/PASSTHROUGH
    if (!can.setRequestedState(node, ODriveState::CLOSED_LOOP)) return false;
    if (!can.setControllerMode(node, CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH)) return false;

    active = true;
    return true;
  }

  // Call frequently from loop()
  void tick() {
    if (!active) return;

    const uint64_t now_us = can.wallTimeUs();

    // Send any frames whose time has arrived (burst out catch-up frames too)
    while (idx < n) {
      // frames[idx].t_s is small (±few seconds), so float→int64 conversion is precise.
      // Adding to uint64_t t_offset_us preserves full microsecond accuracy at any epoch.
      const int64_t frame_off_us = (int64_t)llroundf(frames[idx].t_s * 1e6f);
      const uint64_t t_us = (uint64_t)((int64_t)t_offset_us + frame_off_us);
      if (now_us < t_us) break;

      // Layer C: one-shot settled confirm, evaluated when the FIRST (wind-up) frame
      // is due — the last instant before any hand motion. Reads only cached state
      // (no blocking CAN reads), so a pass adds zero latency to the throw.
      if (require_settled_) {
        require_settled_ = false;  // one-shot regardless of outcome
        CanInterface::AxisHeartbeat hb;
        const bool have_hb = can.getAxisHeartbeat(pitch_node_, hb);
        const unsigned long hb_age_ms =
            (unsigned long)(can.axisHeartbeatMonoAgeUs(pitch_node_) / 1000ULL);
        const bool pitch_done = have_hb && hb.trajectory_done;
        YawAxis::Telemetry yt = yaw_ ? yaw_->readTelemetry() : YawAxis::Telemetry{};
        const float yaw_rate_dps = yt.vel_rps * 360.0f;
        const bool yaw_ok = (yaw_ != nullptr) &&
                            fabsf(yt.err_deg) <= yaw_err_tol_ &&
                            fabsf(yaw_rate_dps) <= yaw_rate_tol_;
        if (!(pitch_done && yaw_ok)) {
          const char* who = (!pitch_done && !yaw_ok) ? "PITCH+YAW"
                          : (!pitch_done ? "PITCH" : "YAW");
          if (Serial) Serial.printf(
              "[Gate] Throw ABORTED — %s not settled: pitch_state=%lu err=0x%lx done=%d "
              "hb_age=%lu ms | yaw_err=%.2f rate=%.1f (tol %.1f,%.1f)\n",
              who, (unsigned long)hb.axis_state, (unsigned long)hb.axis_error,
              (int)pitch_done, hb_age_ms,
              (double)yt.err_deg, (double)yaw_rate_dps,
              (double)yaw_err_tol_, (double)yaw_rate_tol_);
          aborted_ = true;
          active = false;
          return;  // send NO frames — hand stays at rest, ball retained
        }
        // Diagnostic: confirm what "settled" looked like at fire (logbook 2026-06-21).
        if (Serial) Serial.printf(
            "[Gate] settled-confirm OK: pitch_state=%lu hb_age=%lu ms yaw_err=%.2f yaw_rate=%.1f\n",
            (unsigned long)hb.axis_state, hb_age_ms, (double)yt.err_deg, (double)yaw_rate_dps);
      }

      (void)can.sendInputPos(node, frames[idx].pos_cmd, frames[idx].vel_ff, frames[idx].tor_ff);
      ++idx;
    }

    // Finished? → IDLE
    if (idx >= n) {
      (void)can.setRequestedState(node, ODriveState::IDLE);
      active = false;
    }
  }

  bool isActive() const { return active; }
  bool wasAborted() const { return aborted_; }
};
