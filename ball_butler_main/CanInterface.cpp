#include "CanInterface.h"
#include "BallButlerConfig.h"
#include <string.h>
#include <math.h>
#include "Proprioception.h"
#include "StateMachine.h"
#include "Trajectory.h"  // For TrajCfg::LINEAR_GAIN, accelToTorque()
#include "RobotState.h"
#include "FwUpdate.h"     // firmware update over CAN (0x7D6)

// ---------------- Command-name table (optional; PROGMEM friendly) -------------
static const CanInterface::CmdName kCmdNames[] PROGMEM = {
  { CanInterface::Cmd::heartbeat_message, "heartbeat_message" },
  { CanInterface::Cmd::get_error, "get_error" },
  { CanInterface::Cmd::RxSdo, "RxSdo" },
  { CanInterface::Cmd::TxSdo, "TxSdo" },
  { CanInterface::Cmd::set_requested_state, "set_requested_state" },
  { CanInterface::Cmd::get_encoder_estimate, "get_encoder_estimate" },
  { CanInterface::Cmd::set_controller_mode, "set_controller_mode" },
  { CanInterface::Cmd::set_input_pos, "set_input_pos" },
  { CanInterface::Cmd::set_input_vel, "set_input_vel" },
  { CanInterface::Cmd::set_vel_curr_limits, "set_vel_curr_limits" },
  { CanInterface::Cmd::set_traj_vel_limit, "set_traj_vel_limit" },
  { CanInterface::Cmd::set_traj_acc_limits, "set_traj_acc_limits" },
  { CanInterface::Cmd::get_iq, "get_iq" },
  { CanInterface::Cmd::get_temps, "get_temps" },
  { CanInterface::Cmd::reboot_odrives, "reboot_odrives" },
  { CanInterface::Cmd::get_bus_voltage_current, "get_bus_voltage_current" },
  { CanInterface::Cmd::clear_errors, "clear_errors" },
  { CanInterface::Cmd::set_absolute_position, "set_absolute_position" },
  { CanInterface::Cmd::set_pos_gain, "set_pos_gain" },
  { CanInterface::Cmd::set_vel_gains, "set_vel_gains" },
};

const CanInterface::CmdName* CanInterface::commandNameTable(size_t& count) {
  count = sizeof(kCmdNames) / sizeof(kCmdNames[0]);
  return kCmdNames;
}

// ---------------- Error code name table ----------------
const char* CanInterface::errorCodeToString(BallButlerError err) {
  switch (err) {
    case BallButlerError::NONE:          return "NONE";
    case BallButlerError::RELOAD_FAILED: return "RELOAD_FAILED";
    default:                             return "UNKNOWN";
  }
}

// ---------------- static singleton ----------------
CanInterface* CanInterface::s_instance_ = nullptr;

// FW 8 hand input-scale guard: read order. The two scales come first so the
// verdict never waits on the log-only reads behind them.
const uint16_t CanInterface::kScaleGuardEndpoints_[CanInterface::SCALE_GUARD_READS_] = {
  CanInterface::EndpointIds::CAN_INPUT_TORQUE_SCALE,   // 0: verdict
  CanInterface::EndpointIds::CAN_INPUT_VEL_SCALE,      // 1: verdict
  CanInterface::EndpointIds::CAN_NODE_ID,              // 2: log only
  CanInterface::EndpointIds::FW_VERSION_MAJOR,         // 3: log only
  CanInterface::EndpointIds::FW_VERSION_MINOR,         // 4: log only
  CanInterface::EndpointIds::FW_VERSION_REVISION,      // 5: log only
};

// ---------------- ctor ----------------
CanInterface::CanInterface()
  : can1_() {}


// ---------------- begin ----------------
void CanInterface::begin(uint32_t bitrate) {
  s_instance_ = this;

  can1_.begin();
  can1_.setBaudRate(bitrate);
  can1_.setMaxMB(16);
  can1_.enableFIFO();
  can1_.enableFIFOInterrupt();
  can1_.onReceive(rxTrampoline_);

  // Clear states
  have_offset_ = false;
  wall_offset_us_ = 0;
  stats_.clear();
  nextPrint_us_ = micros64() + PRINT_PERIOD_US_;
  home_required_mask_ = 0;

  // Initialise per-node arrays
  for (int i = 0; i < MAX_NODES; ++i) {
    axes_pv_[i].valid       = false;
    axes_iq_[i].valid       = false;
    hb_[i].valid            = false;
    home_state_[i]          = AxisHomeState::Unhomed;
    last_brake_clear_us_[i] = 0;
    arb_param_resp_[i].valid = false;
  }

  // Initialise Ball Butler heartbeat state
  last_heartbeat_ms_ = 0;
  current_error_code_ = BallButlerError::NONE;

  // FW 8 hand input-scale guard: idle; feedforward enabled until a check says
  // otherwise (the first hand arm is the boot homing, which sends no scaled FF).
  scale_guard_phase_ = ScaleGuardPhase::IDLE;
  scale_guard_requested_ = false;
  scale_guard_verdict_ = ScaleGuardVerdict::UNKNOWN;
  for (int i = 0; i < MAX_NODES; ++i) ff_disabled_[i] = false;
}

// ---------------- loop ----------------
void CanInterface::loop() {
  can1_.events();           // dispatch FIFO -> rxTrampoline_
  maybeRunHandScaleGuard_(); // FW 8: hand input-scale SDO readback (async)
  maybePrintSyncStats_();   // optional stats print
  maybePublishHeartbeat_(); // Ball Butler heartbeat publishing
  maybePublishYawEstimate_(); // stamped yaw estimate, one frame per fresh yaw sample
  maybeCheckBallInHand_();  // check for ball in hand
}

// ---------------- debug ----------------
void CanInterface::setDebugStream(Stream* dbg) { dbg_ = dbg; }
void CanInterface::setDebugFlags(bool timeSyncDebug, bool canDebug) {
  dbg_time_ = timeSyncDebug;
  dbg_can_ = canDebug;
}

// ---------------- time functions ----------------
uint64_t CanInterface::localTimeUs() const { return micros64(); }
uint64_t CanInterface::wallTimeUs() const {
  return uint64_t(int64_t(localTimeUs()) + wall_offset_us_);
}

CanInterface::SyncStats CanInterface::getAndClearSyncStats() {
  SyncStats out;
  if (stats_.n) {
    out.mean_us = float(stats_.sum) / float(stats_.n);
    out.rms_us = sqrtf(float(stats_.sum_sq) / float(stats_.n));
    out.min_us = stats_.minv;
    out.max_us = stats_.maxv;
    out.frames = stats_.n;
  }
  stats_.clear();
  return out;
}

// ---------------- generic send ----------------
bool CanInterface::sendRaw(uint32_t id, const uint8_t* data, uint8_t len) {
  CAN_message_t m;
  m.id = id;
  m.len = len;
  m.flags.extended = 0;
  m.flags.remote = 0;
  if (len && data) memcpy(m.buf, data, len);
  const bool ok = can1_.write(m);
  if (dbg_ && dbg_can_) dbg_->printf("[CAN->] id=0x%03lX len=%u %s\n", (unsigned long)id, (unsigned)len, ok ? "OK" : "FAIL");
  return ok;
}

bool CanInterface::sendRTR(uint32_t id, uint8_t len) {
  CAN_message_t m;
  m.id = id;
  m.len = len;
  m.flags.extended = 0;
  m.flags.remote = 1;
  const bool ok = can1_.write(m);
  if (dbg_can_ && dbg_) dbg_->printf("[CAN->RTR] id=0x%03lX len=%u %s\n", (unsigned long)id, (unsigned)len, ok ? "OK" : "FAIL");
  return ok;
}

// ---------------- Handling Jetson Commands ----------------

// ---------------- ODrive helpers ----------------
static inline void wrFloatLE(uint8_t* b, float f) { memcpy(b, &f, 4); }
static inline void wrU32LE(uint8_t* b, uint32_t v) {
  b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF;
}
static inline void wrU16LE(uint8_t* b, uint16_t v) { b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; }

int16_t CanInterface::clampToI16_(float x) {
  if (x > 32767.f) return 32767;
  if (x < -32768.f) return -32768;
  return (int16_t)lrintf(x);
}

bool CanInterface::setRequestedState(uint32_t node_id, uint32_t requested_state) {
  uint8_t d[8] = { 0 };
  wrU32LE(&d[0], requested_state);
  const bool ok = sendRaw(makeId(node_id, Cmd::set_requested_state), d, 8);
  // FW 8: every hand arm (homing, reload, trajectory streamer, ...) re-runs the
  // input-scale guard. Only a flag here: the reads go out from loop(), so this
  // call never adds frames to a caller's own command burst. An arm while a
  // check is already in flight is covered by that check.
  if (ok && node_id == hand_node_id_ && requested_state == ODriveState::CLOSED_LOOP &&
      scale_guard_phase_ == ScaleGuardPhase::IDLE) {
    scale_guard_requested_ = true;
  }
  return ok;
}

bool CanInterface::setControllerMode(uint32_t node_id, uint32_t control_mode, uint32_t input_mode) {
  uint8_t d[8] = { 0 };
  wrU32LE(&d[0], control_mode);
  wrU32LE(&d[4], input_mode);
  return sendRaw(makeId(node_id, Cmd::set_controller_mode), d, 8);
}

bool CanInterface::setVelCurrLimits(uint32_t node_id, float current_limit, float vel_limit_rps) {
  uint8_t d[8];
  wrFloatLE(&d[0], vel_limit_rps);
  wrFloatLE(&d[4], current_limit);
  return sendRaw(makeId(node_id, Cmd::set_vel_curr_limits), d, 8);
}

bool CanInterface::setTrajVelLimit(uint32_t node_id, float vel_limit_rps) {
  uint8_t d[8] = { 0 };
  wrFloatLE(&d[0], vel_limit_rps);
  return sendRaw(makeId(node_id, Cmd::set_traj_vel_limit), d, 8);
}

bool CanInterface::setTrajAccLimits(uint32_t node_id, float accel_rps2, float decel_rps2) {
  uint8_t d[8];
  wrFloatLE(&d[0], accel_rps2);
  wrFloatLE(&d[4], decel_rps2);
  return sendRaw(makeId(node_id, Cmd::set_traj_acc_limits), d, 8);
}

bool CanInterface::setPosGain(uint32_t node_id, float kp) {
  uint8_t d[8] = { 0 };
  wrFloatLE(&d[0], kp);
  return sendRaw(makeId(node_id, Cmd::set_pos_gain), d, 8);
}

bool CanInterface::setVelGains(uint32_t node_id, float kv, float ki) {
  uint8_t d[8];
  wrFloatLE(&d[0], kv);
  wrFloatLE(&d[4], ki);
  return sendRaw(makeId(node_id, Cmd::set_vel_gains), d, 8);
}

bool CanInterface::setAbsolutePosition(uint32_t node_id, float pos_rev) {
  uint8_t d[8] = { 0 };
  wrFloatLE(&d[0], pos_rev);
  return sendRaw(makeId(node_id, Cmd::set_absolute_position), d, 8);
}

bool CanInterface::clearErrors(uint32_t node_id) {
  uint8_t d[8] = { 0 };
  return sendRaw(makeId(node_id, Cmd::clear_errors), d, 8);
}

bool CanInterface::reboot(uint32_t node_id) {
  uint8_t d[8] = { 0 };
  return sendRaw(makeId(node_id, Cmd::reboot_odrives), d, 8);
}

bool CanInterface::sendInputPos(uint32_t node_id, float pos_rev, float vel_ff_rev_per_s, float torque_ff) {
  if (!isAxisMotionAllowed(node_id)) {
    if (dbg_) dbg_->printf("[Gate] Blocked set_input_pos to node %lu (not homed)\n", (unsigned long)node_id);
    return false;
  }
  uint8_t d[8];
  wrFloatLE(&d[0], pos_rev);
  // Per-node scales: pitch has its own pair; every other node (in practice
  // only the hand) uses the hand pair, as all nodes did before FW 7.
  const bool is_pitch = (node_id == pitch_node_id_);
  const float vel_scale = is_pitch ? kPitchVelScale_ : kHandVelScale_;
  const float tor_scale = is_pitch ? kPitchTorScale_ : kHandTorScale_;
  // FW 8: a drive whose CAN input scales read back different from ours gets
  // no feedforward at all (position only) until a check reads them equal.
  const bool ff_off = isFeedforwardDisabled(node_id);
  const int16_t vel_i = ff_off ? int16_t(0) : clampToI16_(vel_ff_rev_per_s * vel_scale);
  const int16_t tor_i = ff_off ? int16_t(0) : clampToI16_(torque_ff * tor_scale);
  d[4] = uint8_t(vel_i & 0xFF);
  d[5] = uint8_t((vel_i >> 8) & 0xFF);
  d[6] = uint8_t(tor_i & 0xFF);
  d[7] = uint8_t((tor_i >> 8) & 0xFF);
  const bool ok = sendRaw(makeId(node_id, Cmd::set_input_pos), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->ODrive pos] node=%lu pos=%.4f vel_ff=%.3f tor_ff=%.3f ok=%d\n",
                 (unsigned long)node_id, (double)pos_rev, (double)vel_ff_rev_per_s, (double)torque_ff, (int)ok);
  }
  return ok;
}

bool CanInterface::sendInputVel(uint32_t node_id, float vel_rps, float torque_ff) {
  if (!isAxisMotionAllowed(node_id)) {
    if (dbg_) dbg_->printf("[Gate] Blocked set_input_vel to node %lu (not homed)\n", (unsigned long)node_id);
    return false;
  }
  uint8_t d[8];
  wrFloatLE(&d[0], vel_rps);
  wrFloatLE(&d[4], torque_ff);
  const bool ok = sendRaw(makeId(node_id, Cmd::set_input_vel), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->ODrive vel] node=%lu vel=%.4f tor_ff=%.3f ok=%d\n",
                 (unsigned long)node_id, (double)vel_rps, (double)torque_ff, (int)ok);
  }
  return ok;
}

// ================================================================================
// Arbitrary Parameter (SDO) Functions
// ================================================================================

bool CanInterface::sendArbitraryParameterFloat(uint32_t node_id, uint16_t endpoint_id, float value) {
  uint8_t d[8];
  d[0] = OPCODE_WRITE;
  wrU16LE(&d[1], endpoint_id);
  d[3] = 0;
  wrFloatLE(&d[4], value);
  const bool ok = sendRaw(makeId(node_id, Cmd::RxSdo), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->SDO Write] node=%lu endpoint=%u value=%.4f ok=%d\n",
                 (unsigned long)node_id, (unsigned)endpoint_id, (double)value, (int)ok);
  }
  return ok;
}

bool CanInterface::sendArbitraryParameterU32(uint32_t node_id, uint16_t endpoint_id, uint32_t value) {
  uint8_t d[8];
  d[0] = OPCODE_WRITE;
  wrU16LE(&d[1], endpoint_id);
  d[3] = 0;
  wrU32LE(&d[4], value);
  const bool ok = sendRaw(makeId(node_id, Cmd::RxSdo), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->SDO Write U32] node=%lu endpoint=%u value=%lu ok=%d\n",
                 (unsigned long)node_id, (unsigned)endpoint_id, (unsigned long)value, (int)ok);
  }
  return ok;
}

bool CanInterface::requestArbitraryParameter(uint32_t node_id, uint16_t endpoint_id) {
  uint8_t d[8] = { 0 };
  d[0] = OPCODE_WRITE; // ODrive expects a write with no data for read requests
  d[1] = endpoint_id & 0xFF;
  d[2] = (endpoint_id >> 8) & 0xFF;
  d[3] = 0;

  return sendRaw(makeId(node_id, Cmd::RxSdo), d, 8);
}

// RxSdo READ, mirroring Jugglebot can-bridge odrive_protocol.h encode_sdo_read:
// [OPCODE_READ][endpoint_id u16 LE][0][0 0 0 0]. The TxSdo reply carries the
// endpoint id in bytes 1-2 and the value in bytes 4-7 (decode_sdo_response_u32).
bool CanInterface::requestArbitraryParameterRead(uint32_t node_id, uint16_t endpoint_id) {
  uint8_t d[8] = { 0 };
  d[0] = OPCODE_READ;
  wrU16LE(&d[1], endpoint_id);
  d[3] = 0;
  return sendRaw(makeId(node_id, Cmd::RxSdo), d, 8);
}

bool CanInterface::getLastArbitraryParamResponse(uint32_t node_id, ArbitraryParamResponse& out) const {
  if (node_id >= MAX_NODES) return false;
  ArbitraryParamResponse snap;
  uint64_t t1, t2;
  do {
    t1 = arb_param_resp_[node_id].wall_us;
    snap = arb_param_resp_[node_id];
    t2 = arb_param_resp_[node_id].wall_us;
  } while (t1 != t2);
  if (!snap.valid) return false;
  out = snap;
  return true;
}

void CanInterface::setArbitraryParamCallback(ArbitraryParamCallback cb, void* user) {
  arb_param_cb_ = cb;
  arb_param_cb_user_ = user;
}

bool CanInterface::isEncoderSearchComplete(uint32_t node_id, uint32_t timeout_ms) {
  if (!requestArbitraryParameter(node_id, EndpointIds::COMMUTATION_MAPPER_POS_ABS)) return false;
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    can1_.events();
    ArbitraryParamResponse resp;
    if (getLastArbitraryParamResponse(node_id, resp)) {
      if (resp.endpoint_id == EndpointIds::COMMUTATION_MAPPER_POS_ABS) {
        return !isnan(resp.value.f32);
      }
    }
    delay(1);
  }
  if (dbg_) dbg_->printf("[SDO] Timeout waiting for encoder search status on node %lu\n", (unsigned long)node_id);
  return false;
}

bool CanInterface::readGpioStates(uint32_t node_id, uint32_t& states_out, uint32_t timeout_ms) {
  if (!requestArbitraryParameter(node_id, EndpointIds::GPIO_STATES)) {
    if (dbg_) dbg_->printf("[SDO] Failed to send GPIO states request to node %lu\n", (unsigned long)node_id);
    return false;
  }
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    can1_.events();
    ArbitraryParamResponse resp;
    if (getLastArbitraryParamResponse(node_id, resp)) {
      if (resp.endpoint_id == EndpointIds::GPIO_STATES) {
        states_out = resp.value.u32;
        return true;
      }
    }
    delay(1);
  }
  if (dbg_) dbg_->printf("[SDO] Timeout waiting for GPIO states on node %lu\n", (unsigned long)node_id);
  return false;
}

// ================================================================================
// Operating Config and Axis Access
// ================================================================================

bool CanInterface::restoreHandToOperatingConfig(uint32_t node_id, float vel_limit_rps, float current_limit_A) {
  bool ok = true;
  ok &= setRequestedState(node_id, ODriveState::IDLE);
  ok &= setControllerMode(node_id, ODriveControlMode::POSITION, ODriveInputMode::PASSTHROUGH);
  ok &= setVelCurrLimits(node_id, current_limit_A, vel_limit_rps);
  return ok;
}

bool CanInterface::getAxisPV(uint32_t node_id, float& pos_out, float& vel_out, uint64_t& wall_us_out) const {
  if (node_id >= MAX_NODES) return false;
  uint64_t t1, t2;
  float p, v;
  do {
    t1 = axes_pv_[node_id].wall_us;
    p = axes_pv_[node_id].pos_rev;
    v = axes_pv_[node_id].vel_rps;
    t2 = axes_pv_[node_id].wall_us;
  } while (t1 != t2);
  if (!axes_pv_[node_id].valid) return false;
  pos_out = p; vel_out = v; wall_us_out = t1;
  return true;
}

uint64_t CanInterface::axisPVMonoAgeUs(uint32_t node_id) const {
  if (node_id >= MAX_NODES || !axes_pv_[node_id].valid) return UINT64_MAX;
  const uint64_t m = axes_pv_[node_id].mono_us;
  const uint64_t now = micros64();
  return (now >= m) ? (now - m) : 0;   // monotonic: never underflows
}

uint64_t CanInterface::axisHeartbeatMonoAgeUs(uint32_t node_id) const {
  if (node_id >= MAX_NODES || !hb_[node_id].valid) return UINT64_MAX;
  const uint64_t m = hb_[node_id].mono_us;
  const uint64_t now = micros64();
  return (now >= m) ? (now - m) : 0;   // monotonic: never underflows
}

bool CanInterface::getAxisIq(uint32_t node_id, float& iq_meas_out, float& iq_setp_out, uint64_t& wall_us_out) const {
  if (node_id >= MAX_NODES) return false;
  uint64_t t1, t2;
  float iqm, iqs;
  do {
    t1 = axes_iq_[node_id].wall_us;
    iqm = axes_iq_[node_id].iq_meas;
    iqs = axes_iq_[node_id].iq_setp;
    t2 = axes_iq_[node_id].wall_us;
  } while (t1 != t2);
  if (!axes_iq_[node_id].valid) return false;
  iq_meas_out = iqm; iq_setp_out = iqs; wall_us_out = t1;
  return true;
}

// ================================================================================
// Homing
// ================================================================================

// --- Non-blocking homing implementation ---

bool CanInterface::startHomeHand(uint32_t node_id, float homing_speed_rps, float current_limit_A,
                                 float current_headroom_A, float settle_pos_rev, float avg_weight) {
  // Store parameters for use across phases
  homing_node_id_    = node_id;
  homing_speed_rps_  = homing_speed_rps;
  homing_current_A_  = current_limit_A;
  homing_headroom_A_ = current_headroom_A;
  homing_settle_rev_ = settle_pos_rev;
  homing_ema_weight_ = constrain(avg_weight, 0.f, 0.9999f);
  homing_ema_        = 0.0f;
  homing_last_iq_us_ = 0;
  homing_start_ms_   = millis();
  homing_phase_ms_   = millis();

  // Begin: set home state and enter CONFIGURE phase
  setHomeState(node_id, AxisHomeState::Homing);
  homing_phase_ = HomingPhase::CONFIGURE;
  return true;
}

bool CanInterface::startHomeHandStandard(uint32_t node_id, int hand_direction, float base_speed_rps,
                                         float current_limit_A, float current_headroom_A, float set_abs_pos_rev) {
  const float homing_speed = float(hand_direction) * base_speed_rps;
  return startHomeHand(node_id, homing_speed, current_limit_A, current_headroom_A, set_abs_pos_rev,
                       HandDefaults::HOMING_EMA_WEIGHT);
}

CanInterface::HomingStatus CanInterface::updateHomeHand() {
  if (homing_phase_ == HomingPhase::IDLE) return HomingStatus::DONE;

  const uint32_t node_id = homing_node_id_;

  // Per-attempt timeout — applied to ALL phases including MONITOR_IQ.
  // This ensures a failed attempt (e.g. ODrive never entered CLOSED_LOOP)
  // is detected and retried rather than silently consuming the entire
  // overall timeout budget.
  if (millis() - homing_start_ms_ > HandDefaults::HOMING_ATTEMPT_TIMEOUT_MS) {
    if (dbg_) dbg_->println("[Home] Per-attempt timeout while homing hand.");
    setRequestedState(node_id, ODriveState::IDLE);
    setHomeState(node_id, AxisHomeState::Unhomed);
    homing_phase_ = HomingPhase::IDLE;
    return HomingStatus::FAILED;
  }

  switch (homing_phase_) {
    // ----------------------------------------------------------------
    case HomingPhase::CONFIGURE: {
      // Clear any stale errors from previous attempts or power-cycle boot.
      // The CLOSED_LOOP request is deferred to CLEAR_SETTLE so the ODrive
      // has time to process the error clear first.
      clearErrors(node_id);
      homing_phase_ = HomingPhase::CLEAR_SETTLE;
      homing_phase_ms_ = millis();
      return HomingStatus::IN_PROGRESS;
    }

    // ----------------------------------------------------------------
    case HomingPhase::CLEAR_SETTLE: {
      // Wait for the ODrive to process clearErrors before sending the
      // state transition.  Without this gap the CLOSED_LOOP request can
      // arrive before the error clear takes effect, causing it to be
      // silently rejected.
      static constexpr uint32_t CLEAR_SETTLE_MS = 20;
      if (millis() - homing_phase_ms_ < CLEAR_SETTLE_MS) {
        return HomingStatus::IN_PROGRESS;
      }
      // Now send CLOSED_LOOP, velocity control mode, and current/vel limits
      if (!setRequestedState(node_id, ODriveState::CLOSED_LOOP) ||
          !setControllerMode(node_id, ODriveControlMode::VELOCITY, ODriveInputMode::VEL_RAMP)) {
        setHomeState(node_id, AxisHomeState::Unhomed);
        homing_phase_ = HomingPhase::IDLE;
        return HomingStatus::FAILED;
      }
      const float vel_limit = fabsf(homing_speed_rps_ * 2.0f);
      if (!setVelCurrLimits(node_id, homing_current_A_ + homing_headroom_A_, vel_limit)) {
        setHomeState(node_id, AxisHomeState::Unhomed);
        homing_phase_ = HomingPhase::IDLE;
        return HomingStatus::FAILED;
      }
      homing_phase_ = HomingPhase::SETTLE;
      homing_phase_ms_ = millis();
      return HomingStatus::IN_PROGRESS;
    }

    // ----------------------------------------------------------------
    case HomingPhase::SETTLE: {
      // Wait for control mode to take effect before checking state
      if (millis() - homing_phase_ms_ < HandDefaults::HOMING_MODE_SETTLE_MS) {
        return HomingStatus::IN_PROGRESS;
      }
      // Verify the ODrive actually entered CLOSED_LOOP before proceeding.
      // If the heartbeat still shows IDLE (or an error), keep waiting —
      // the per-attempt timeout will catch persistent failures and allow
      // a retry rather than silently hanging.
      AxisHeartbeat hb;
      if (getAxisHeartbeat(node_id, hb)) {
        if (hb.axis_error != 0) {
          if (dbg_) dbg_->printf("[Home] Axis error during settle (err=0x%08lX)\n",
                                  (unsigned long)hb.axis_error);
          setHomeState(node_id, AxisHomeState::Unhomed);
          homing_phase_ = HomingPhase::IDLE;
          return HomingStatus::FAILED;
        }
        if (hb.axis_state == ODriveState::CLOSED_LOOP) {
          homing_phase_ = HomingPhase::SEND_VEL;
          return HomingStatus::IN_PROGRESS;
        }
      }
      // Not yet confirmed — keep waiting (per-attempt timeout will catch failure)
      return HomingStatus::IN_PROGRESS;
    }

    // ----------------------------------------------------------------
    case HomingPhase::SEND_VEL: {
      if (!sendInputVel(node_id, homing_speed_rps_, 0.0f)) {
        setRequestedState(node_id, ODriveState::IDLE);
        setHomeState(node_id, AxisHomeState::Unhomed);
        homing_phase_ = HomingPhase::IDLE;
        return HomingStatus::FAILED;
      }
      if (dbg_) dbg_->printf("[Home] node=%lu moving at %.3f rps; Iq limit=%.2f A\n",
                              (unsigned long)node_id, (double)homing_speed_rps_, (double)homing_current_A_);
      homing_phase_ = HomingPhase::MONITOR_IQ;
      homing_phase_ms_ = millis();
      return HomingStatus::IN_PROGRESS;
    }

    // ----------------------------------------------------------------
    case HomingPhase::MONITOR_IQ: {
      // Independent heartbeat check — detect ODrive falling out of
      // CLOSED_LOOP even when no Iq data is arriving.  The SETTLE phase
      // already confirmed CLOSED_LOOP before we got here, so any
      // heartbeat showing IDLE or an error is a genuine failure.
      AxisHeartbeat hb;
      if (getAxisHeartbeat(node_id, hb)) {
        if (hb.axis_state == ODriveState::IDLE || hb.axis_error != 0) {
          if (dbg_) dbg_->printf("[Home] Axis error or unexpected IDLE during monitoring (state=%u, err=0x%08lX)\n",
                                  (unsigned)hb.axis_state, (unsigned long)hb.axis_error);
          setHomeState(node_id, AxisHomeState::Unhomed);
          homing_phase_ = HomingPhase::IDLE;
          return HomingStatus::FAILED;
        }
      }

      // Check for fresh Iq reading and apply EMA filter
      float iq_meas, iq_setp;
      uint64_t t_us;
      if (getAxisIq(node_id, iq_meas, iq_setp, t_us)) {
        if (t_us != homing_last_iq_us_) {
          homing_last_iq_us_ = t_us;
          homing_ema_ = homing_ema_weight_ * homing_ema_ + (1.f - homing_ema_weight_) * iq_meas;

          if (fabsf(homing_ema_) >= homing_current_A_) {
            // Current spike detected — stop the motor
            setRequestedState(node_id, ODriveState::IDLE);
            homing_phase_ = HomingPhase::STOP_SETTLE;
            homing_phase_ms_ = millis();
            return HomingStatus::IN_PROGRESS;
          }
        }
      }
      return HomingStatus::IN_PROGRESS;
    }

    // ----------------------------------------------------------------
    case HomingPhase::STOP_SETTLE: {
      // Wait for motor to stop before setting absolute position
      if (millis() - homing_phase_ms_ < HandDefaults::HOMING_STOP_SETTLE_MS) {
        return HomingStatus::IN_PROGRESS;
      }
      homing_phase_ = HomingPhase::FINALIZE;
      return HomingStatus::IN_PROGRESS;
    }

    // ----------------------------------------------------------------
    case HomingPhase::FINALIZE: {
      setAbsolutePosition(node_id, homing_settle_rev_);
      if (dbg_) dbg_->printf("[Home] node=%lu homed. Set pos to %.3f rev.\n",
                              (unsigned long)node_id, (double)homing_settle_rev_);
      restoreHandToOperatingConfig(node_id);
      setHomeState(node_id, AxisHomeState::Homed);
      homing_phase_ = HomingPhase::IDLE;
      return HomingStatus::DONE;
    }

    default:
      homing_phase_ = HomingPhase::IDLE;
      return HomingStatus::FAILED;
  }
}

// ================================================================================
// Home State Management
// ================================================================================

void CanInterface::setHomeState(uint32_t node_id, AxisHomeState s) {
  if (node_id < MAX_NODES) home_state_[node_id] = s;
}

CanInterface::AxisHomeState CanInterface::getHomeState(uint32_t node_id) const {
  return (node_id < MAX_NODES) ? home_state_[node_id] : AxisHomeState::Unhomed;
}

void CanInterface::requireHomeForAxis(uint32_t node_id, bool required) {
  if (node_id >= MAX_NODES) return;
  const uint64_t bit = (1ULL << node_id);
  if (required) home_required_mask_ |= bit;
  else home_required_mask_ &= ~bit;
}

bool CanInterface::isHomeRequired(uint32_t node_id) const {
  if (node_id >= MAX_NODES) return false;
  return (home_required_mask_ & (1ULL << node_id)) != 0;
}

void CanInterface::requireHomeOnlyFor(uint32_t node_id) {
  home_required_mask_ = (node_id < MAX_NODES) ? (1ULL << node_id) : 0;
}

void CanInterface::clearHomeRequirements() { home_required_mask_ = 0; }

bool CanInterface::isAxisMotionAllowed(uint32_t node_id) const {
  if (node_id >= MAX_NODES) return false;
  if (!isHomeRequired(node_id)) return true;
  AxisHomeState s = home_state_[node_id];
  return s == AxisHomeState::Homing || s == AxisHomeState::Homed;
}

// ================================================================================
// Axis Heartbeat
// ================================================================================

bool CanInterface::getAxisHeartbeat(uint32_t node_id, AxisHeartbeat& out) const {
  if (node_id >= MAX_NODES) return false;
  AxisHeartbeat snap;
  uint64_t t1, t2;
  do {
    t1 = hb_[node_id].wall_us;
    snap = hb_[node_id];
    t2 = hb_[node_id].wall_us;
  } while (t1 != t2);
  if (!snap.valid) return false;
  out = snap;
  return true;
}

bool CanInterface::hasAxisError(uint32_t node_id, uint32_t mask) const {
  AxisHeartbeat hb;
  return getAxisHeartbeat(node_id, hb) && (hb.axis_error & mask);
}

bool CanInterface::waitForAxisErrorClear(uint32_t node_id, uint32_t mask, uint32_t timeout_ms, uint16_t poll_ms) {
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    loop();
    AxisHeartbeat hb;
    if (getAxisHeartbeat(node_id, hb) && (hb.axis_error & mask) == 0u) return true;
    delay(poll_ms);
  }
  return false;
}

void CanInterface::setAutoClearBrakeResistor(bool enable, uint32_t min_interval_ms) {
  auto_clear_brake_res_ = enable;
  auto_clear_interval_ms_ = min_interval_ms ? min_interval_ms : 1;
}

void CanInterface::setEstimatorCmd(uint8_t cmd) { estimatorCmd_ = (cmd & 0x1F); }

// ================================================================================
// Ball Butler Heartbeat
// ================================================================================
void CanInterface::maybePublishHeartbeat_() {
  if (heartbeat_rate_ms_ == 0) return;
  if (!state_machine_) return;
  const uint32_t now_ms = millis();
  if (now_ms - last_heartbeat_ms_ < heartbeat_rate_ms_) return;
  last_heartbeat_ms_ = now_ms;
  publishHeartbeat_();
}

void CanInterface::publishHeartbeat_() {
  RobotState state = state_machine_->getState();
  
  // Map RobotState to heartbeat state values
  uint8_t state_val = robotStateToUint8(state);
  
  // Byte 0: State byte (bit 0 = ball_in_hand, bits 1-7 = state)
  uint8_t state_byte = (ball_in_hand_ ? 0x01 : 0x00) | (state_val << 1);
  
  // Byte 1: State data (error_code for ERROR, 0 otherwise)
  uint8_t state_data = 0;
  if (state == RobotState::ERROR) {
    state_data = static_cast<uint8_t>(current_error_code_);
  }
  
  // Position feedback from Proprioception
  ProprioceptionData prop;
  PRO.snapshot(prop);
  
  // Yaw: resolution ~0.01° : 0-360° -> 0-65535 (uint16),
  uint16_t yaw_enc = 0;
  if (prop.isYawValid()) {
    float yaw_deg = fmodf(prop.yaw_deg, 360.0f);
    if (yaw_deg < 0) yaw_deg += 360.0f;
    yaw_enc = (uint16_t)(yaw_deg / HeartbeatCfg::YAW_RES_DEG);
  }

  // Pitch:  resolution ~0.002° : 0-131.072° -> 0-65535 (uint16),
  uint16_t pitch_enc = 0;
  if (prop.isPitchValid()) {
    float pitch_deg = constrain(prop.pitch_deg, HeartbeatCfg::PITCH_CLAMP_MIN, HeartbeatCfg::PITCH_CLAMP_MAX);
    pitch_enc = (uint16_t)(pitch_deg / HeartbeatCfg::PITCH_RES_DEG);
  }

  // Hand: resolution 0.01mm : 0-655.36 mm -> 0-65535 (uint16),
  uint16_t hand_enc = 0;
  if (prop.isHandPVValid()) {
    float hand_mm = prop.hand_pos_rev / TrajCfg::LINEAR_GAIN * 1000.0f;
    hand_mm = constrain(hand_mm, 0.0f, HeartbeatCfg::HAND_MAX_MM);
    hand_enc = (uint16_t)(hand_mm / HeartbeatCfg::HAND_RES_MM);
  }
  
  // Assemble frame (little-endian for uint16 fields)
  uint8_t frame[8];
  frame[0] = state_byte;
  frame[1] = state_data;
  frame[2] = yaw_enc & 0xFF;
  frame[3] = (yaw_enc >> 8) & 0xFF;
  frame[4] = pitch_enc & 0xFF;
  frame[5] = (pitch_enc >> 8) & 0xFF;
  frame[6] = hand_enc & 0xFF;
  frame[7] = (hand_enc >> 8) & 0xFF;

  // Use centralized CAN ID from CanIds namespace
  sendRaw(CanIds::HEARTBEAT_CMD, frame, 8);
}

// ================================================================================
// Stamped yaw estimate — YAW_ESTIMATE (0x7D8), 2026-10-09
// ================================================================================
// The 10 Hz heartbeat yaw reaches the host unstamped through three free-running
// 10 Hz stages (a per-session 78-174 ms lag against mocap). This frame carries
// the same Proprioception yaw once per FRESH YawAxis sample (150 Hz ISR), plus
// how old that sample is at TX, so the can-bridge can stamp it at sample time
// and forward it beside pitch/hand in BB_AXIS_ESTIMATES -> /bb/axis_estimates.
//
// Layout (8 bytes, little-endian):
//   bytes 0-3 = yaw_deg  (float32, BB-local degrees, unwrapped — the heartbeat
//               value before its [0,360) wrap and 0.01 deg truncation)
//   bytes 4-5 = yaw_vel  (int16, deg/s / YawEstimateEncoding::vel_res_dps)
//   bytes 6-7 = age_us   (uint16, micros64() at TX minus the sample's ISR
//               micros64() stamp, saturating at 65535) — monotonic, no wall
//               clock, so it is immune to time-sync slews
// The heartbeat is untouched; its other consumers keep their 10 Hz yaw.
void CanInterface::maybePublishYawEstimate_() {
  float yaw_deg = 0.f, yaw_vel_rps = 0.f;
  uint64_t ts_us = 0;
  if (!PRO.getYawPV(yaw_deg, yaw_vel_rps, ts_us)) return;   // no yaw sample yet
  if (ts_us == last_yaw_est_ts_us_) return;                  // already sent this sample
  last_yaw_est_ts_us_ = ts_us;

  const uint64_t now_us = micros64();
  const uint64_t age64  = (now_us > ts_us) ? (now_us - ts_us) : 0;
  const uint16_t age_us = (age64 > 65535u) ? 65535u : (uint16_t)age64;
  const int16_t  vel_i  = clampToI16_(yaw_vel_rps * 360.0f / YawEstimateEncoding::vel_res_dps);

  uint8_t frame[8];
  memcpy(&frame[0], &yaw_deg, 4);          // Teensy 4 is little-endian
  frame[4] = uint8_t(vel_i & 0xFF);
  frame[5] = uint8_t((vel_i >> 8) & 0xFF);
  frame[6] = uint8_t(age_us & 0xFF);
  frame[7] = uint8_t((age_us >> 8) & 0xFF);
  sendRaw(CanIds::YAW_ESTIMATE, frame, 8);
}

// ================================================================================
// Loud command-outcome channel (Phase 2) — CMD_RESULT (0x7D5)
// ================================================================================
bool CanInterface::publishCmdResult(uint8_t cmd_type, uint8_t outcome,
                                    int16_t detail0, int16_t detail1) {
  // Little-endian int16 details, mirroring the heartbeat frame idiom.
  const uint16_t d0 = (uint16_t)detail0;
  const uint16_t d1 = (uint16_t)detail1;
  uint8_t frame[8];
  frame[0] = cmd_type;
  frame[1] = outcome;
  frame[2] = d0 & 0xFF;
  frame[3] = (d0 >> 8) & 0xFF;
  frame[4] = d1 & 0xFF;
  frame[5] = (d1 >> 8) & 0xFF;
  frame[6] = 0;  // reserved
  frame[7] = 0;  // reserved
  return sendRaw(CanIds::CMD_RESULT, frame, 8);
}

// ================================================================================
// Ball in Hand Check
// ================================================================================
void CanInterface::maybeCheckBallInHand_() {
  // Only check in states where ball presence matters
  if (!state_machine_) return;
  const RobotState st = state_machine_->getState();
  if (st != RobotState::IDLE && st != RobotState::TRACKING && st != RobotState::CHECKING_BALL && st != RobotState::RELOADING) return;

  switch (ball_check_phase_) {
    case BallCheckPhase::IDLE: {
      if (millis() - last_ball_check_ms_ < ball_check_interval_ms_) return;
      // Clear cached response so we detect the fresh reply
      if (hand_node_id_ < MAX_NODES) arb_param_resp_[hand_node_id_].valid = false;
      // Send async SDO request for GPIO states (non-blocking)
      if (!requestArbitraryParameter(hand_node_id_, EndpointIds::GPIO_STATES)) {
        if (dbg_) dbg_->printf("[CAN] Ball check: failed to send GPIO request\n");
        last_ball_check_ms_ = millis();  // Back off before retrying
        return;
      }
      ball_check_phase_ = BallCheckPhase::WAITING;
      ball_check_sent_ms_ = millis();
      break;
    }
    case BallCheckPhase::WAITING: {
      // Check for response (non-blocking)
      ArbitraryParamResponse resp;
      if (getLastArbitraryParamResponse(hand_node_id_, resp) &&
          resp.endpoint_id == EndpointIds::GPIO_STATES) {
        // Got response — process GPIO states
        const uint32_t gpio_states = resp.value.u32;
        ball_in_hand_ = !((gpio_states >> ball_detect_gpio_pin_) & 0x01);

        if (ball_in_hand_) {
          ball_false_count_ = 0;
        } else {
          ball_false_count_++;
          if (ball_false_count_ >= max_ball_missing_samples_) {
            if (dbg_) dbg_->printf("[CAN] %d consecutive ball-missing readings, entering CHECKING_BALL\n", ball_false_count_);
            state_machine_->requestCheckBall();
            ball_false_count_ = 0;
          }
        }

        last_ball_check_ms_ = millis();
        ball_check_phase_ = BallCheckPhase::IDLE;
      } else if (millis() - ball_check_sent_ms_ > BALL_CHECK_TIMEOUT_MS) {
        // Timeout — no response received
        if (dbg_) dbg_->printf("[CAN] Ball check: GPIO read timeout\n");
        last_ball_check_ms_ = millis();
        ball_check_phase_ = BallCheckPhase::IDLE;
      }
      // Otherwise: still waiting, return without blocking
      break;
    }
  }
}

// ================================================================================
// RX Handlers
// ================================================================================

void CanInterface::rxTrampoline_(const CAN_message_t& msg) {
  if (s_instance_) s_instance_->handleRx_(msg);
}

void CanInterface::handleRx_(const CAN_message_t& msg) {
  // Time sync - uses instance variable (can be overridden from default)
  if (msg.id == CanIds::TIME_SYNC_CMD && msg.len == 8 && !msg.flags.remote) {
    handleTimeSync_(msg);
    return;
  }

  // Firmware update over CAN (FwUpdate.h). Handled before anything else: the
  // id has no other meaning, and a DATA burst must not walk the checks below.
  if (msg.id == CanIds::FW_UPDATE_CMD && !msg.flags.remote) {
    FwUpdate::handleCommand(msg);
    return;
  }

  // While a firmware-update session is open the robot is parked and the state
  // machine frozen until the reboot that ends the session: refuse every host
  // motion/state command rather than queue one that could act on a parked axis.
  if (FwUpdate::sessionOpen() &&
      (msg.id == CanIds::HOST_THROW_CMD || msg.id == CanIds::RELOAD_CMD ||
       msg.id == CanIds::RESET_CMD || msg.id == CanIds::CALIBRATE_LOC_CMD)) {
    if (dbg_ && dbg_can_) {
      dbg_->printf("[CAN] id=0x%03lX ignored: firmware-update session open\n", (unsigned long)msg.id);
    }
    return;
  }

  const uint32_t node = (msg.id >> 5);
  const uint8_t cmd = msg.id & 0x1F;

  // Host -> Teensy throw command
  if (msg.id == CanIds::HOST_THROW_CMD && msg.len == 8 && !msg.flags.remote) {
    const int16_t yaw_i = (int16_t)((uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8));
    const uint16_t pit_u = (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
    const uint16_t sp_u = (uint16_t)msg.buf[4] | ((uint16_t)msg.buf[5] << 8);
    const uint16_t t_u = (uint16_t)msg.buf[6] | ((uint16_t)msg.buf[7] << 8);

    HostThrowCmd c;
    c.yaw_rad = float(yaw_i) * (float)M_PI / 32768.0f;
    c.pitch_rad = float(pit_u) * ((float)M_PI / 65536.0f);
    c.speed_mps = float(sp_u) * 0.0001f;

    // Decode absolute throw time from lower 16 bits of epoch milliseconds.
    // Reconstruct full uint64_t using current wall clock for the upper bits.
    {
      const uint64_t now_ms = wallTimeUs() / 1000ULL;
      const uint64_t base   = now_ms & ~0xFFFFULL;
      uint64_t thr_ms       = base | (uint64_t)t_u;
      // Handle wrap-around: throw should be in the near future
      if (now_ms > thr_ms + 32768ULL) thr_ms += 65536ULL;
      c.throw_wall_us = thr_ms * 1000ULL;
    }

    if (c.yaw_rad < -M_PI) c.yaw_rad = -M_PI;
    if (c.yaw_rad > M_PI) c.yaw_rad = M_PI;
    if (c.pitch_rad < 0.f) c.pitch_rad = 0.f;
    const float PI_2 = (float)M_PI * 0.5f;
    if (c.pitch_rad > PI_2) c.pitch_rad = PI_2;
    if (c.speed_mps < 0.f) c.speed_mps = 0.f;
    if (c.speed_mps > 6.5535f) c.speed_mps = 6.5535f;

    c.wall_us = wallTimeUs();
    c.valid = true;
    last_host_cmd_ms_ = millis();  // Track local time for idle timeout

    if (dbg_ && dbg_can_) {
      const float lead_ms = (float)((int64_t)c.throw_wall_us - (int64_t)c.wall_us) / 1000.0f;
      dbg_->printf("[HostCmd] yaw=%.3f rad pitch=%.3f rad speed=%.3f m/s throw_in=%.0f ms (id=0x%03lX)\n",
                   (double)c.yaw_rad, (double)c.pitch_rad, (double)c.speed_mps,
                   (double)lead_ms, (unsigned long)msg.id);
    }

    // Route to StateMachine based on speed
    if (state_machine_) {
      // Convert radians to degrees for StateMachine
      const float yaw_deg = c.yaw_rad * (180.0f / (float)M_PI);
      const float pitch_deg = c.pitch_rad * (180.0f / (float)M_PI);

      if (c.speed_mps == 0.0f) {
        // Tracking mode: just update yaw/pitch targets (no throw)
        state_machine_->requestTracking(yaw_deg, pitch_deg);
      } else {
        // Throw mode: queue a throw with absolute wall-clock time
        state_machine_->requestThrow(yaw_deg, pitch_deg, c.speed_mps, c.throw_wall_us);
      }
    }

    return;
  }

  // Host -> BB RELOAD command
  if (msg.id == CanIds::RELOAD_CMD && msg.len == 0 && !msg.flags.remote) {
    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-RELOAD] id=0x%03lX\n", (unsigned long)msg.id);
    }
    if (state_machine_) {
      state_machine_->requestCheckBall();
    }
    return;
  }

  // Host -> BB RESET command
  if (msg.id == CanIds::RESET_CMD && msg.len == 0 && !msg.flags.remote) {
    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-RESET] id=0x%03lX\n", (unsigned long)msg.id);
    }
    if (state_machine_) {
      state_machine_->reset();
    }
    return;
  }

  // Host -> BB CALIBRATE LOCATION command
  if (msg.id == CanIds::CALIBRATE_LOC_CMD && msg.len == 0 && !msg.flags.remote) {
    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-CALIBRATE_LOC] id=0x%03lX\n", (unsigned long)msg.id);
    }
    if (state_machine_) {
      state_machine_->requestCalibrateLocation();
    }
    return;
  }

  // ODrive heartbeat
  if (cmd == uint8_t(Cmd::heartbeat_message) && node < MAX_NODES && msg.len >= 7 && !msg.flags.remote) {
    uint32_t axis_error = (uint32_t)msg.buf[0] | ((uint32_t)msg.buf[1] << 8)
                        | ((uint32_t)msg.buf[2] << 16) | ((uint32_t)msg.buf[3] << 24);

    hb_[node].axis_error = axis_error;
    hb_[node].axis_state = msg.buf[4];
    hb_[node].procedure_result = msg.buf[5];
    hb_[node].trajectory_done = msg.buf[6];
    hb_[node].wall_us = wallTimeUs();
    hb_[node].mono_us = micros64();
    hb_[node].valid = true;

    if (auto_clear_brake_res_ && (axis_error & ODriveErrors::BRAKE_RESISTOR_DISARMED)) {
      const uint64_t now_us = micros64();
      const uint64_t min_gap_us = (uint64_t)auto_clear_interval_ms_ * 1000ULL;
      if (now_us - last_brake_clear_us_[node] >= min_gap_us) {
        last_brake_clear_us_[node] = now_us;
        const bool ok = clearErrors(node);
        if (dbg_) {
          dbg_->printf("[AutoClear] node=%lu axis_error=0x%08lX -> Clear_Errors %s\n",
                       (unsigned long)node, (unsigned long)axis_error, ok ? "OK" : "FAIL");
        }
      }
    }
    return;
  }

  // TxSdo (arbitrary parameter response)
  if (cmd == uint8_t(Cmd::TxSdo) && node < MAX_NODES) {
    handleTxSdo_(node, msg.buf, msg.len);
    return;
  }

  // Estimator (pos, vel)
  if (cmd == estimatorCmd_ && msg.len == 8 && node < MAX_NODES && !msg.flags.remote) {
    float pos, vel;
    memcpy(&pos, &msg.buf[0], 4);
    memcpy(&vel, &msg.buf[4], 4);
    axes_pv_[node].pos_rev = pos;
    axes_pv_[node].vel_rps = vel;
    axes_pv_[node].wall_us = wallTimeUs();
    axes_pv_[node].mono_us = micros64();   // monotonic stamp for sync-immune freshness
    axes_pv_[node].valid = true;

    Proprioception& prop = PRO;
    const uint64_t t_us = axes_pv_[node].wall_us;

    if (node == hand_node_id_) {
      prop.setHandPV(pos, vel, t_us);
    }
    if (node == pitch_node_id_) {
      const float pitch_deg = 90.0f + pos * 360.0f;
      prop.setPitchDeg(pitch_deg, t_us);
    }

    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-PV] node=%lu pos=%.4f rev vel=%.4f rps id=0x%03lX\n",
                   (unsigned long)node, (double)pos, (double)vel, (unsigned long)msg.id);
    }
    return;
  }

  // Iq feedback
  if (cmd == uint8_t(Cmd::get_iq) && msg.len == 8 && node < MAX_NODES && !msg.flags.remote) {
    float iq_meas, iq_setp;
    memcpy(&iq_meas, &msg.buf[0], 4);
    memcpy(&iq_setp, &msg.buf[4], 4);
    axes_iq_[node].iq_meas = iq_meas;
    axes_iq_[node].iq_setp = iq_setp;
    axes_iq_[node].wall_us = wallTimeUs();
    axes_iq_[node].valid = true;

    if (node == hand_node_id_) {
      PRO.setHandIq(iq_meas, axes_iq_[node].wall_us);
    }

    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-Iq] node=%lu Iq=%.3fA set=%.3fA id=0x%03lX\n",
                   (unsigned long)node, (double)iq_meas, (double)iq_setp, (unsigned long)msg.id);
    }
    return;
  }

  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN] id=0x%03lX len=%d rtr=%d\n", (unsigned long)msg.id, msg.len, (int)msg.flags.remote);
  }
}

void CanInterface::handleTxSdo_(uint32_t node_id, const uint8_t* buf, uint8_t len) {
  if (len < 8) return;
  const uint8_t opcode = buf[0];
  const uint16_t endpoint_id = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);

  // FW 8: the scale guard's replies are consumed here and never reach
  // arb_param_resp_, so they cannot displace the ball-in-hand poll's reply.
  uint32_t value_u32;
  memcpy(&value_u32, &buf[4], 4);
  if (scaleGuardTakeReply_(node_id, endpoint_id, value_u32)) return;

  ArbitraryParamResponse resp;
  resp.opcode = opcode;
  resp.endpoint_id = endpoint_id;
  resp.wall_us = wallTimeUs();
  memcpy(&resp.value.f32, &buf[4], 4);
  resp.valid = true;
  arb_param_resp_[node_id] = resp;
  
  if (arb_param_cb_) {
    arb_param_cb_(node_id, resp, arb_param_cb_user_);
  }
}

void CanInterface::handleTimeSync_(const CAN_message_t& msg) {
  uint32_t sec = (uint32_t)msg.buf[0] | ((uint32_t)msg.buf[1] << 8) 
               | ((uint32_t)msg.buf[2] << 16) | ((uint32_t)msg.buf[3] << 24);
  uint32_t usec = (uint32_t)msg.buf[4] | ((uint32_t)msg.buf[5] << 8) 
                | ((uint32_t)msg.buf[6] << 16) | ((uint32_t)msg.buf[7] << 24);

  const uint64_t master_us = (uint64_t)sec * 1'000'000ULL + (uint64_t)usec;
  const uint64_t local_us = micros64();
  const int64_t offset = (int64_t)master_us - (int64_t)local_us;

  if (!have_offset_) {
    wall_offset_us_ = offset;
    have_offset_ = true;
  } else {
    const int64_t diff = offset - wall_offset_us_;
    wall_offset_us_ += (diff >> ALPHA_SHIFT_);
  }

  const int32_t residual = int32_t(offset - wall_offset_us_);
  stats_.add(residual);
}

void CanInterface::maybePrintSyncStats_() {
  if (!dbg_time_ || !dbg_) return;
  const uint64_t now = micros64();
  if (now < nextPrint_us_) return;
  nextPrint_us_ = now + PRINT_PERIOD_US_;
  if (!stats_.n) return;

  const float mean = float(stats_.sum) / float(stats_.n);
  const float rms = sqrtf(float(stats_.sum_sq) / float(stats_.n));
  dbg_->printf("[TimeSync] mean=%+.1f us  rms=%.1f us  min=%+d  max=%+d  n=%lu\n",
               (double)mean, (double)rms, stats_.minv, stats_.maxv, (unsigned long)stats_.n);
  stats_.clear();
}
// ================================================================================
// FW 8 hand input-scale guard
// ================================================================================
// FW 6 sent the hand ODrive (S1, node 8, input_torque_scale 100) torque
// feedforward at Jugglebot's scale 1000: 10x. FW 7 gave BB its own scales;
// this checks them against the drive itself at every hand arm:
//   - both scales read back == bb_hand_tor / bb_hand_vel -> feedforward on
//     (clears a latch); INFO printed when the verdict becomes MATCH;
//   - either reads back different                       -> latch ff_disabled_
//     for the node at once (sendInputPos sends vel_ff = tor_ff = 0); loud
//     warning every check while it persists;
//   - a scale does not answer within SCALE_GUARD_READ_TIMEOUT_MS -> latch left
//     as it was (on, unless an earlier check latched it off); warning.
// Non-blocking and bounded: one RxSdo in flight, fixed arrays, the whole check
// ends within SCALE_GUARD_TOTAL_MS. The endpoint ids are the S1 0.6.11-1 table's
// (EndpointId::odrive_s1_0_6_11); the fw version is read and printed (not
// gated on) so the log shows which build answered.

bool CanInterface::scaleGuardTakeReply_(uint32_t node_id, uint16_t endpoint_id, uint32_t value) {
  if (scale_guard_phase_ == ScaleGuardPhase::IDLE) return false;
  if (node_id != hand_node_id_) return false;
  for (uint8_t i = 0; i < SCALE_GUARD_READS_; ++i) {
    if (kScaleGuardEndpoints_[i] != endpoint_id) continue;
    scale_guard_val_[i] = value;
    scale_guard_got_mask_ |= uint8_t(1u << i);
    // A mismatching scale disables feedforward immediately, not at the end of
    // the check; only a check in which both match clears it (finish).
    if ((i == 0 && value != kHandTorScaleU32_) || (i == 1 && value != kHandVelScaleU32_)) {
      ff_disabled_[node_id] = true;
    }
    return true;
  }
  return false;
}

void CanInterface::maybeRunHandScaleGuard_() {
  if (hand_node_id_ >= MAX_NODES) return;
  const uint32_t now = millis();

  if (scale_guard_phase_ == ScaleGuardPhase::IDLE) {
    if (!scale_guard_requested_) return;
    scale_guard_requested_ = false;
    scale_guard_idx_ = 0;
    scale_guard_got_mask_ = 0;
    for (uint8_t i = 0; i < SCALE_GUARD_READS_; ++i) scale_guard_val_[i] = 0;
    scale_guard_start_ms_ = now;
    scale_guard_phase_ = ScaleGuardPhase::SEND;
  }

  if (now - scale_guard_start_ms_ > SCALE_GUARD_TOTAL_MS) {
    finishHandScaleGuard_();
    return;
  }

  if (scale_guard_phase_ == ScaleGuardPhase::SEND) {
    // A full TX queue just retries next loop (bounded by the total window).
    if (requestArbitraryParameterRead(hand_node_id_, kScaleGuardEndpoints_[scale_guard_idx_])) {
      scale_guard_sent_ms_ = now;
      scale_guard_phase_ = ScaleGuardPhase::WAIT;
    }
    return;
  }

  // WAIT: next read on the reply, or after the per-read timeout.
  const bool got = (scale_guard_got_mask_ >> scale_guard_idx_) & 0x01;
  if (!got && (now - scale_guard_sent_ms_) <= SCALE_GUARD_READ_TIMEOUT_MS) return;
  if (++scale_guard_idx_ >= SCALE_GUARD_READS_) {
    finishHandScaleGuard_();
    return;
  }
  scale_guard_phase_ = ScaleGuardPhase::SEND;
}

void CanInterface::finishHandScaleGuard_() {
  scale_guard_phase_ = ScaleGuardPhase::IDLE;
  const uint32_t node = hand_node_id_;
  const uint8_t m = scale_guard_got_mask_;
  const bool got_tor = m & 0x01, got_vel = m & 0x02;
  const uint32_t tor = scale_guard_val_[0], vel = scale_guard_val_[1];
  const bool bad = (got_tor && tor != kHandTorScaleU32_) || (got_vel && vel != kHandVelScaleU32_);
  const ScaleGuardVerdict prev = scale_guard_verdict_;

  ScaleGuardVerdict v;
  if (bad) {
    v = ScaleGuardVerdict::MISMATCH;
    ff_disabled_[node] = true;
  } else if (got_tor && got_vel) {
    v = ScaleGuardVerdict::MATCH;
    ff_disabled_[node] = false;
  } else {
    v = ScaleGuardVerdict::NO_REPLY;   // ff_disabled_ unchanged
  }
  scale_guard_verdict_ = v;

  if (!dbg_) return;
  if (v == ScaleGuardVerdict::MATCH && prev == ScaleGuardVerdict::MATCH) return;  // once per change

  char tor_s[12], vel_s[12], nid_s[12], fw_s[16];
  if (got_tor) snprintf(tor_s, sizeof(tor_s), "%lu", (unsigned long)tor); else snprintf(tor_s, sizeof(tor_s), "no-reply");
  if (got_vel) snprintf(vel_s, sizeof(vel_s), "%lu", (unsigned long)vel); else snprintf(vel_s, sizeof(vel_s), "no-reply");
  if (m & 0x04) snprintf(nid_s, sizeof(nid_s), "%lu", (unsigned long)scale_guard_val_[2]); else snprintf(nid_s, sizeof(nid_s), "?");
  if ((m & 0x38) == 0x38) {
    snprintf(fw_s, sizeof(fw_s), "%u.%u.%u", (unsigned)(scale_guard_val_[3] & 0xFF),
             (unsigned)(scale_guard_val_[4] & 0xFF), (unsigned)(scale_guard_val_[5] & 0xFF));
  } else {
    snprintf(fw_s, sizeof(fw_s), "?");
  }

  if (v == ScaleGuardVerdict::MATCH) {
    dbg_->printf("[ScaleGuard] INFO hand node %lu input_torque_scale=%s input_vel_scale=%s "
                 "(expected %lu/%lu) MATCH - feedforward ON | drive node_id=%s fw=%s\n",
                 (unsigned long)node, tor_s, vel_s, (unsigned long)kHandTorScaleU32_,
                 (unsigned long)kHandVelScaleU32_, nid_s, fw_s);
  } else if (v == ScaleGuardVerdict::MISMATCH) {
    dbg_->printf("[ScaleGuard] !!!!! WARNING hand node %lu CAN input scale MISMATCH: drive "
                 "input_torque_scale=%s input_vel_scale=%s, BB sends at %lu/%lu (bb_hand_tor/bb_hand_vel). "
                 "vel_ff and torque_ff ZEROED for node %lu until both read back equal | drive node_id=%s fw=%s\n",
                 (unsigned long)node, tor_s, vel_s, (unsigned long)kHandTorScaleU32_,
                 (unsigned long)kHandVelScaleU32_, (unsigned long)node, nid_s, fw_s);
  } else {
    dbg_->printf("[ScaleGuard] WARNING hand node %lu input-scale readback incomplete "
                 "(input_torque_scale=%s input_vel_scale=%s within %lu ms per read) - feedforward left %s | "
                 "drive node_id=%s fw=%s\n",
                 (unsigned long)node, tor_s, vel_s, (unsigned long)SCALE_GUARD_READ_TIMEOUT_MS,
                 ff_disabled_[node] ? "OFF (earlier mismatch)" : "ON (unverified)", nid_s, fw_s);
  }
}
