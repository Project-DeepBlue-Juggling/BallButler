#pragma once
/*
 * FwUpdate.h - Firmware update over CAN (FlasherX-style self-reprogramming)
 * ============================================================================
 *
 * A port of the Platform Teensy's receiver (Jugglebot repo,
 * ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
 * § FIRMWARE UPDATE OVER CAN), which flashed Platform FW 6 over CAN on
 * 2026-09-09. Same MCU (Teensy 4.0), same wire contract, same staging and
 * commit code; only the CAN ids and the "is it safe to start" step differ.
 * The host side is Jugglebot's tools/teensy_link_bridge.py --fw-update
 * --target bb, relayed by the can-bridge's BB_FW_* RPCs on CAN1.
 *
 * WIRE CONTRACT (little-endian throughout; the Platform's, verbatim):
 *
 *   Command, id BallButlerCanId::FW_UPDATE_CMD (0x7D6), byte 0 = opcode
 *     0x01 BEGIN   [op][image_len u32 @1..4][0,0,0]
 *     0x02 DATA    [op][seq u16 @1..2][payload @3..7], n = dlc-3, 1..5 bytes
 *     0x03 VERIFY  [op][crc32 u32 @1..4][0,0,0]
 *     0x04 COMMIT  [op][0...]
 *     0x05 INFO    [op][0...]                      <- Ball Butler only
 *
 *   Reply, id BallButlerCanId::FW_UPDATE_REPLY (0x7D7), dlc 8
 *     [opcode echoed @0][status @1][seq u16 @2..3][detail u32 @4..7]
 *     seq    = last accepted DATA seq, or the EXPECTED seq on BAD_SEQ
 *     detail = staged byte count (BEGIN/DATA), computed crc (VERIFY),
 *              staging capacity (TOO_BIG), FW_VERSION (INFO),
 *              pitch angle in signed centidegrees (PARKING / PARK_FAILED), else 0
 *
 *   DATA is strictly in order; ACK (OK) on every 16th accepted frame
 *   (seq % 16 == 15) and on the frame that completes the image; out-of-order
 *   frames are NAKed BAD_SEQ with the expected seq and the host rewinds. A 4 KB
 *   sector flush runs with interrupts off, so frames arriving during it are
 *   lost from the CAN hardware FIFO; the host pauses after each sector and the
 *   BAD_SEQ rewind is the designed recovery for anything it misses.
 *
 * PARKING (Ball Butler only). A sector erase disables interrupts for up to
 * ~400 ms, and the yaw PID runs in an IntervalTimer ISR, so the yaw motor
 * would keep its last PWM with no control loop. So the first accepted BEGIN
 * latches a session and parks the robot before any flash is touched:
 *   1. yaw e-stopped (PWM latched at 0 — safe even while the ISR is blocked);
 *   2. pitch moved to its stow angle (>= SMDefaults::PITCH_MIN_STOW_DEG, 80
 *      deg) if it is below it — below ~70 deg an IDLE pitch drops;
 *   3. once pitch is stowed and its trajectory done: pitch and hand ODrives
 *      IDLE, confirmed by their heartbeats.
 * Until parked, BEGIN answers PARKING and the host re-sends it. A park that
 * does not complete within PARK_TIMEOUT_MS answers PARK_FAILED and reboots.
 *
 * EVERY SESSION ENDS IN A REBOOT (owner's call, 2026-09-28). A COMMIT reboots
 * into the new image; an abandoned session (60 s without a command — a
 * --verify-only run, a crashed host, a failed VERIFY) and a failed park reboot
 * too, so Ball Butler always comes back through BOOT with every axis freshly
 * initialised and never from a half-restored park. While a session is open
 * the state machine and streamer are frozen (see loop()) and host motion
 * commands are refused.
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>

class CanInterface;
class PitchAxis;
class YawAxis;
class StateMachine;
class HandTrajectoryStreamer;

namespace FwUpdate {

// Firmware identity. FW_NAME is the marker VERIFY requires in a staged image
// (and the host tool checks before sending one): it separates a Ball Butler
// build from a Platform ("jugglebot-platform") or can-bridge image that
// happened to CRC correctly. Bump FW_VERSION on every flashed change — the
// INFO reply is the only receipt that a COMMIT landed. Jugglebot pins the
// expected value in teensy_link/rpc_args.py BB_FW_VERSION_EXPECTED.
constexpr char     FW_NAME[]  = "ballbutler-main";
constexpr uint16_t FW_VERSION = 4;   // 1: 2026-09-28 first image with the CAN receiver (USB-flashed)
                                     // 2: 2026-09-28 no code change — the first CAN-flashed image; the version is the receipt
                                     // 3: 2026-09-28 no code change — receipt for the faster transfer (host sector pause 0.5 -> 0.12 s)
                                     // 4: 2026-09-28 no code change — receipt for the pipelined transfer (bridge FW 22, depth 4)

// Wire the receiver to the hardware it parks. Call first thing in setup(),
// before canif.begin(): until attached, every command is ignored.
void attach(CanInterface& can, PitchAxis& pitch, YawAxis& yaw,
            StateMachine& sm, HandTrajectoryStreamer& streamer);

// Handle one FW_UPDATE_CMD frame (from CanInterface's RX dispatch, main-loop
// context: FlexCAN_T4 queues in the ISR and dispatches from events()).
void handleCommand(const CAN_message_t& msg);

// Call every loop(): advances the park and times out an abandoned session.
void tick();

// True from the first accepted BEGIN until the reboot that ends the session.
bool sessionOpen();

}  // namespace FwUpdate
