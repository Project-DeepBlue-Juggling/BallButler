// FwUpdate.cpp - see FwUpdate.h for the wire contract, parking and the
// reboot-ends-every-session rule. The staging / CRC / identity / commit code
// is the Platform receiver's, unchanged but for naming.
#include "FwUpdate.h"

#include "BallButlerConfig.h"
#include "CanInterface.h"
#include "PitchAxis.h"
#include "YawAxis.h"
#include "StateMachine.h"
#include "HandTrajectoryStreamer.h"

extern "C" {
/*  cores/teensy4/eeprom.c — "To be called from LittleFS_Program, any other use
 *  at your own risk!".  Both mask interrupts internally and RE-ENABLE them on
 *  the way out (via that file's flash_wait()), which is why the commit loop
 *  re-masks after every single call rather than once at the top. */
void eepromemu_flash_write(void *addr, const void *data, uint32_t len);
void eepromemu_flash_erase_sector(void *addr);
}

/*  Linker symbol from cores/teensy4/imxrt1062.ld:
 *      _flashimagelen = __text_csf_end - ORIGIN(FLASH);
 *  i.e. the byte length of the RUNNING image.  Its ADDRESS is the value. */
extern unsigned long _flashimagelen;

namespace FwUpdate {

constexpr uint32_t FLASH_BASE   = 0x60000000u;
constexpr uint32_t SECTOR_SIZE  = 4096u;
constexpr uint32_t WRITE_CHUNK  = 256u;  // one QSPI page — LittleFS_Program's prog_size,
                                         // so a proven length for eepromemu_flash_write
/*  Top of usable program flash = base of the core's EEPROM-emulation reserve
 *  (eeprom.c FLASH_BASEADDR under ARDUINO_TEENSY40, = ORIGIN(FLASH) + 1984K).
 *  Ball Butler does not use EEPROM, but the reserve is the linker's boundary
 *  all the same: nothing may be staged at or above it. */
constexpr uint32_t EEPROM_RESERVE = 0x601F0000u;

constexpr uint32_t COMMIT_DRAIN_MS   = 50;      // let the last reply reach the bridge
constexpr uint32_t SESSION_IDLE_MS   = 60000;   // abandoned-session escape hatch → reboot

// Parking (see FwUpdate.h).
constexpr uint32_t PARK_TIMEOUT_MS     = 10000;   // < the host's 20 s, so PARK_FAILED wins
constexpr uint32_t PARK_RESEND_MS      = 200;     // re-request IDLE until the heartbeats agree
constexpr uint64_t PARK_FRESH_US       = 300000;  // pitch PV + heartbeats must be this fresh

/*  Opcodes (host → board, byte 0 of a FW_UPDATE_CMD frame). */
constexpr uint8_t OP_BEGIN = 0x01, OP_DATA = 0x02, OP_VERIFY = 0x03, OP_COMMIT = 0x04,
                  OP_INFO = 0x05;

/*  Status codes (byte 1 of the reply).  THE PLATFORM RECEIVER, THE CAN-BRIDGE
 *  RELAY AND THE HOST TOOL SHARE THIS TABLE — never renumber, only append.
 *  8 and 9 were appended for Ball Butler; the Platform never sends them. */
constexpr uint8_t ST_OK           = 0;
constexpr uint8_t ST_BUSY         = 1;  // state machine not IDLE/ERROR, or the streamer active (BEGIN)
constexpr uint8_t ST_BAD_STATE    = 2;  // no session open, wrong phase, or malformed dlc
constexpr uint8_t ST_BAD_SEQ      = 3;  // out-of-order DATA; seq field = the expected seq
constexpr uint8_t ST_TOO_BIG      = 4;  // image_len > staging capacity, or DATA past image_len
constexpr uint8_t ST_BAD_CRC      = 5;
constexpr uint8_t ST_BAD_IDENTITY = 6;  // staged image is not a ballbutler-main build
constexpr uint8_t ST_FLASH_ERR    = 7;  // staging write failed its read-back
constexpr uint8_t ST_PARKING      = 8;  // BEGIN: parking in progress — re-send BEGIN
constexpr uint8_t ST_PARK_FAILED  = 9;  // BEGIN: park timed out; the board reboots

/*  Transfer state.  IDLE → (BEGIN, once parked) → RECEIVING → (VERIFY ok) →
 *  VERIFIED → (COMMIT).  A failed VERIFY drops back to RECEIVING; a failed
 *  staging write drops to IDLE (the session stays latched: only a reboot or a
 *  fresh BEGIN moves on).  BEGIN resets the transfer from anywhere. */
enum Phase : uint8_t { P_IDLE = 0, P_RECEIVING = 1, P_VERIFIED = 2 };

/*  Park state — one-way until reboot. */
enum Park : uint8_t { PK_NONE = 0, PK_MOVING = 1, PK_IDLING = 2, PK_PARKED = 3 };

static CanInterface*           s_can      = nullptr;
static PitchAxis*              s_pitch    = nullptr;
static YawAxis*                s_yaw      = nullptr;
static StateMachine*           s_sm       = nullptr;
static HandTrajectoryStreamer* s_streamer = nullptr;

static Phase    phase         = P_IDLE;
static Park     park          = PK_NONE;
static uint32_t image_len     = 0;   // declared at BEGIN
static uint32_t stage_base    = 0;   // computed at BEGIN
static uint32_t stage_cap     = 0;   // EEPROM_RESERVE - stage_base
static uint32_t flushed       = 0;   // image bytes already committed to the staging flash
static uint32_t buf_fill      = 0;   // image bytes held in sector_buf
static uint16_t expect_seq    = 0;   // next DATA seq we will accept
static uint16_t last_seq      = 0;   // last DATA seq accepted (reported in replies)
static uint32_t last_cmd_ms   = 0;   // millis() of the last command frame, for the idle timeout
static uint32_t park_start_ms = 0;
static uint32_t park_resend_ms = 0;

/*  One 4 KB sector, reused for staging AND for the commit copy (the commit reads
 *  a staged sector into here before erasing its destination — the flash cannot
 *  be read while it is being programmed). */
alignas(4) static uint8_t sector_buf[SECTOR_SIZE];

static inline uint32_t staged() { return flushed + buf_fill; }

/*  Staging starts at the first sector boundary ABOVE the running image, so the
 *  staged copy never overlaps the code that is doing the staging. */
static inline uint32_t stagingBase() {
  const uint32_t img_end = FLASH_BASE + (uint32_t)&_flashimagelen;
  return (img_end + (SECTOR_SIZE - 1u)) & ~(SECTOR_SIZE - 1u);
}

static void reply(uint8_t op, uint8_t status, uint16_t seq, uint32_t detail) {
  uint8_t b[8];
  b[0] = op;
  b[1] = status;
  b[2] = uint8_t(seq & 0xFF);
  b[3] = uint8_t(seq >> 8);
  b[4] = uint8_t(detail & 0xFF);
  b[5] = uint8_t((detail >> 8) & 0xFF);
  b[6] = uint8_t((detail >> 16) & 0xFF);
  b[7] = uint8_t((detail >> 24) & 0xFF);
  s_can->sendRaw(CanIds::FW_UPDATE_REPLY, b, 8);
}

static void rebootNow(const char* why) {
  Serial.printf("[fwupd] %s — rebooting\n", why);
  delay(COMMIT_DRAIN_MS);   // let any queued reply reach the bridge
  SCB_AIRCR = 0x05FA0004;   // system reset
  while (1) {}
}

// ── Parking ──────────────────────────────────────────────────────────────────

/*  Pitch angle from the ODrive encoder estimate, fresh on the MONOTONIC clock
 *  (Proprioception's stamp is wall-clock, which the time-sync slews). */
static bool pitchDeg(float& deg) {
  const uint32_t node = NodeId::BB_PITCH;
  if (s_can->axisPVMonoAgeUs(node) > PARK_FRESH_US) return false;
  float pos = 0, vel = 0; uint64_t t = 0;
  if (!s_can->getAxisPV(node, pos, vel, t)) return false;
  deg = PitchAxis::revToDeg(pos);
  return true;
}

static uint32_t pitchCentideg() {
  float deg = 0;
  if (!pitchDeg(deg)) return 0x80000000u;   // INT32_MIN: "no fresh reading"
  return (uint32_t)(int32_t)lroundf(deg * 100.0f);
}

static bool axisIdle(uint32_t node) {
  CanInterface::AxisHeartbeat hb;
  if (s_can->axisHeartbeatMonoAgeUs(node) > PARK_FRESH_US) return false;
  return s_can->getAxisHeartbeat(node, hb) && hb.axis_state == ODriveState::IDLE;
}

static bool pitchStowedAndSettled() {
  float deg = 0;
  if (!pitchDeg(deg) || deg < SMDefaults::PITCH_MIN_STOW_DEG) return false;
  CanInterface::AxisHeartbeat hb;
  if (s_can->axisHeartbeatMonoAgeUs(NodeId::BB_PITCH) > PARK_FRESH_US) return false;
  if (!s_can->getAxisHeartbeat(NodeId::BB_PITCH, hb)) return false;
  // An IDLE pitch at or above the stow angle is already where the park wants
  // it; a CLOSED_LOOP one must have finished its move before it is released.
  return hb.axis_state == ODriveState::IDLE || hb.trajectory_done;
}

static void requestIdle() {
  s_can->setRequestedState(NodeId::BB_PITCH, ODriveState::IDLE);
  s_can->setRequestedState(NodeId::BB_HAND,  ODriveState::IDLE);
  park_resend_ms = millis();
}

/*  Latch the session and start the park. Called once, from the first BEGIN
 *  that passes the state gate. */
static void startPark() {
  park_start_ms = millis();
  s_yaw->estop();                       // PWM latched at 0 before anything else
  float deg = 0;
  if (pitchDeg(deg) && deg >= SMDefaults::PITCH_MIN_STOW_DEG) {
    Serial.printf("[fwupd] park: pitch %.1f deg already stowed — idling axes\n", (double)deg);
  } else {
    Serial.println("[fwupd] park: raising pitch to stow before idling axes");
    s_can->setRequestedState(NodeId::BB_PITCH, ODriveState::CLOSED_LOOP);
    s_pitch->setTargetDeg(SMDefaults::PITCH_DEG_HOME);   // sent by pitch.loop() once CLOSED_LOOP
  }
  park = PK_MOVING;
}

static void advancePark() {
  if (park == PK_NONE || park == PK_PARKED) return;

  if (millis() - park_start_ms > PARK_TIMEOUT_MS) {
    reply(OP_BEGIN, ST_PARK_FAILED, 0, pitchCentideg());
    rebootNow("park timed out");
  }

  if (park == PK_MOVING && pitchStowedAndSettled()) {
    requestIdle();
    park = PK_IDLING;
    return;
  }
  if (park == PK_IDLING) {
    if (axisIdle(NodeId::BB_PITCH) && axisIdle(NodeId::BB_HAND)) {
      park = PK_PARKED;
      Serial.println("[fwupd] parked: yaw e-stopped, pitch + hand IDLE — ready for the image");
    } else if (millis() - park_resend_ms >= PARK_RESEND_MS) {
      requestIdle();
    }
  }
}

// ── Staging / verify / commit (the Platform receiver's, unchanged) ──────────

/*  Erase + write the buffered bytes into the staging flash, then READ THEM BACK.
 *  The read-back is the only way a failed erase or write can be noticed at all:
 *  the core's primitives return void and the QSPI status register is consumed
 *  inside them.  Erasing lazily — here, rather than blanking the whole region at
 *  BEGIN — keeps BEGIN's reply immediate and spreads the stall over the
 *  transfer instead of concentrating a multi-second freeze at its start. */
static bool flushBuffer() {
  if (buf_fill == 0) return true;

  const uint32_t addr = stage_base + flushed;
  const uint32_t n    = (buf_fill + 3u) & ~3u;          // pad up to a 4-byte write
  for (uint32_t i = buf_fill; i < n; ++i) sector_buf[i] = 0xFF;

  eepromemu_flash_erase_sector((void *)addr);
  for (uint32_t off = 0; off < n; off += WRITE_CHUNK) {
    uint32_t len = n - off;
    if (len > WRITE_CHUNK) len = WRITE_CHUNK;
    eepromemu_flash_write((void *)(addr + off), sector_buf + off, len);
  }

  arm_dcache_delete((void *)addr, n);                   // read from flash, not from cache
  if (memcmp((const void *)addr, sector_buf, n) != 0) {
    Serial.printf("[fwupd] FLASH_ERR: read-back mismatch at 0x%08lX\n", (unsigned long)addr);
    return false;
  }

  flushed  += buf_fill;   // padding is NOT image content
  buf_fill  = 0;
  return true;
}

/*  CRC-32/IEEE 802.3 — the zlib / binascii.crc32 flavour the host computes:
 *  reflected polynomial 0xEDB88320, init 0xFFFFFFFF, final xor 0xFFFFFFFF.
 *  Bitwise, so it needs no 1 KB table in RAM; ~20 ms for a 150 KB image. */
static uint32_t crc32(const uint8_t *p, uint32_t n) {
  uint32_t crc = 0xFFFFFFFFu;
  while (n--) {
    crc ^= *p++;
    for (uint8_t k = 0; k < 8; ++k)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
  }
  return ~crc;
}

/*  Identity gate: the staged image must contain this board's own FW_NAME
 *  marker. The CRC proves the transfer; only this proves the INTENT. */
static bool identityOk() {
  const uint8_t *img  = (const uint8_t *)stage_base;
  const uint32_t nlen = sizeof(FW_NAME) - 1u;   // NUL excluded
  if (image_len < nlen) return false;
  for (uint32_t i = 0; i + nlen <= image_len; ++i) {
    if (img[i] == (uint8_t)FW_NAME[0] && memcmp(img + i, FW_NAME, nlen) == 0) return true;
  }
  return false;
}

/*  Copy staged → program flash, sector by sector, low to high, then reboot.
 *  Never returns.
 *
 *  THE OVERLAP INVARIANT.  Destination sector s spans [FLASH_BASE + s*4096,
 *  +4096); its source is [stage_base + s*4096, +4096).  stage_base is at least
 *  one sector above FLASH_BASE, so dst_s + 4096 <= src_s for every s: erasing a
 *  destination sector can only destroy source bytes that were read one iteration
 *  EARLIER, never bytes still to come.  This is the whole reason the copy must
 *  run low to high and must never be reordered.
 *
 *  Each sector is read into RAM before its destination is touched, because the
 *  QSPI cannot serve an AHB read while an erase or page-program is in flight.
 *  Everything on this path runs from ITCM/DTCM, so nothing here reads flash. */
static void commitAndReboot() {
  const uint32_t sectors = (image_len + SECTOR_SIZE - 1u) / SECTOR_SIZE;

  __disable_irq();
  for (uint32_t s = 0; s < sectors; ++s) {
    const uint32_t src = stage_base + s * SECTOR_SIZE;
    const uint32_t dst = FLASH_BASE  + s * SECTOR_SIZE;

    arm_dcache_delete((void *)src, SECTOR_SIZE);
    memcpy(sector_buf, (const void *)src, SECTOR_SIZE);

    __disable_irq();                                  // the primitives re-enable on exit
    eepromemu_flash_erase_sector((void *)dst);
    __disable_irq();
    for (uint32_t off = 0; off < SECTOR_SIZE; off += WRITE_CHUNK) {
      eepromemu_flash_write((void *)(dst + off), sector_buf + off, WRITE_CHUNK);
      __disable_irq();
    }
  }

  SCB_AIRCR = 0x05FA0004;   // system reset
  while (1) {}
}

// ── Public API ───────────────────────────────────────────────────────────────

void attach(CanInterface& can, PitchAxis& pitch, YawAxis& yaw,
            StateMachine& sm, HandTrajectoryStreamer& streamer) {
  s_can = &can; s_pitch = &pitch; s_yaw = &yaw; s_sm = &sm; s_streamer = &streamer;
}

bool sessionOpen() { return park != PK_NONE; }

void handleCommand(const CAN_message_t &msg) {
  if (!s_can || msg.len < 1) return;
  const uint8_t op = msg.buf[0];

  // INFO is a read — no session, no state change, answered any time.
  if (op == OP_INFO) { reply(OP_INFO, ST_OK, 0, FW_VERSION); return; }

  last_cmd_ms = millis();

  switch (op) {

    case OP_BEGIN: {
      if (msg.len < 5) { reply(op, ST_BAD_STATE, 0, 0); return; }
      const uint32_t len = uint32_t(msg.buf[1]) | (uint32_t(msg.buf[2]) << 8)
                         | (uint32_t(msg.buf[3]) << 16) | (uint32_t(msg.buf[4]) << 24);

      // Capacity first: never park the robot for an image that cannot fit.
      stage_base = stagingBase();
      stage_cap  = EEPROM_RESERVE - stage_base;
      if (len == 0 || len > stage_cap) {
        phase = P_IDLE;
        reply(op, ST_TOO_BIG, 0, stage_cap);
        return;
      }

      if (park == PK_NONE) {
        // The state gate applies only to OPENING a session; once latched, the
        // state machine is frozen and its state no longer moves.
        const RobotState st = s_sm->getState();
        if ((st != RobotState::IDLE && st != RobotState::ERROR) || s_streamer->isActive()) {
          Serial.printf("[fwupd] BEGIN refused: state %s%s\n", robotStateToString(st),
                        s_streamer->isActive() ? " (streamer active)" : "");
          reply(op, ST_BUSY, 0, 0);
          return;
        }
        startPark();
      }
      if (park != PK_PARKED) {
        advancePark();                  // may already be done (pitch stowed + idle)
        if (park != PK_PARKED) { reply(op, ST_PARKING, 0, pitchCentideg()); return; }
      }

      image_len  = len;
      flushed    = 0;
      buf_fill   = 0;
      expect_seq = 0;
      last_seq   = 0;
      phase      = P_RECEIVING;
      reply(op, ST_OK, 0, 0);
      Serial.printf("[fwupd] BEGIN %lu B  staging 0x%08lX..0x%08lX (%lu KB free)\n",
                    (unsigned long)image_len, (unsigned long)stage_base,
                    (unsigned long)EEPROM_RESERVE, (unsigned long)(stage_cap / 1024u));
      return;
    }

    case OP_DATA: {
      if (phase != P_RECEIVING)         { reply(op, ST_BAD_STATE, expect_seq, staged()); return; }
      if (msg.len < 4 || msg.len > 8)   { reply(op, ST_BAD_STATE, expect_seq, staged()); return; }

      const uint16_t seq = uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8);
      const uint32_t n   = uint32_t(msg.len) - 3u;
      if (seq != expect_seq)            { reply(op, ST_BAD_SEQ, expect_seq, staged()); return; }
      if (staged() + n > image_len)     { reply(op, ST_TOO_BIG, seq, staged()); return; }

      /* A payload can straddle a sector boundary (n is 1..5 and need not divide
       * 4096), so take it in two bites when it does. */
      uint32_t off = 0;
      while (off < n) {
        uint32_t take = SECTOR_SIZE - buf_fill;
        if (take > n - off) take = n - off;
        memcpy(sector_buf + buf_fill, &msg.buf[3 + off], take);
        buf_fill += take;
        off      += take;
        if (buf_fill == SECTOR_SIZE && !flushBuffer()) {
          phase = P_IDLE;
          reply(op, ST_FLASH_ERR, seq, staged());
          return;
        }
      }

      last_seq   = seq;
      expect_seq = uint16_t(seq + 1u);

      const bool complete = (staged() == image_len);
      if ((seq % 16u) == 15u || complete) reply(op, ST_OK, seq, staged());
      return;
    }

    case OP_VERIFY: {
      if (phase == P_IDLE || msg.len < 5) { reply(op, ST_BAD_STATE, last_seq, 0); return; }

      if (!flushBuffer()) {               // push the tail out before hashing
        phase = P_IDLE;
        reply(op, ST_FLASH_ERR, last_seq, 0);
        return;
      }
      if (staged() != image_len) {        // short image — nothing to verify yet
        phase = P_RECEIVING;
        reply(op, ST_BAD_STATE, expect_seq, staged());
        return;
      }

      const uint32_t want = uint32_t(msg.buf[1]) | (uint32_t(msg.buf[2]) << 8)
                          | (uint32_t(msg.buf[3]) << 16) | (uint32_t(msg.buf[4]) << 24);
      arm_dcache_delete((void *)stage_base, image_len);
      const uint32_t got = crc32((const uint8_t *)stage_base, image_len);

      if (got != want) {
        phase = P_RECEIVING;
        Serial.printf("[fwupd] BAD_CRC want=0x%08lX got=0x%08lX\n",
                      (unsigned long)want, (unsigned long)got);
        reply(op, ST_BAD_CRC, last_seq, got);
        return;
      }
      if (!identityOk()) {
        phase = P_RECEIVING;
        Serial.println("[fwupd] BAD_IDENTITY: staged image carries no FW_NAME marker");
        reply(op, ST_BAD_IDENTITY, last_seq, got);
        return;
      }

      phase = P_VERIFIED;
      Serial.printf("[fwupd] VERIFY ok, crc=0x%08lX — COMMIT armed\n", (unsigned long)got);
      reply(op, ST_OK, last_seq, got);
      return;
    }

    case OP_COMMIT: {
      if (phase != P_VERIFIED) { reply(op, ST_BAD_STATE, last_seq, 0); return; }
      /* Reply BEFORE the copy and let it drain: once the erase starts, this
       * board answers nothing until it comes back up on the new image. The
       * robot is parked (VERIFIED implies PARKED), so there is no motion to
       * interrupt. */
      reply(op, ST_OK, last_seq, 0);
      Serial.println("[fwupd] COMMIT — copying staged image over program flash");
      delay(COMMIT_DRAIN_MS);
      commitAndReboot();       // never returns
      return;
    }

    default:
      reply(op, ST_BAD_STATE, last_seq, 0);
      return;
  }
}

void tick() {
  if (park == PK_NONE) return;
  advancePark();
  if (millis() - last_cmd_ms > SESSION_IDLE_MS) rebootNow("session idle 60 s — aborted");
}

}  // namespace FwUpdate
