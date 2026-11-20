#pragma once

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.0174532925199433f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308233f
#endif

#define CAN_CORE 0x01
#define ARM_CORE 0x02
#define IMU_CORE 0x01

#define CAN_SHOULDER_SET_YZ_DEGREE 0x01
#define CAN_ELBOW_SET_Y_DEGREE 0x02
#define CAN_WRIST_SET_YZ_DEGREE 0x03
#define CAN_CLAW_SET_XYG_DEGREE 0x04
#define CAN_SHOULDER_FIRMWARE_UPGRADE 0x05
#define CAN_ELBOW_FIRMWARE_UPGRADE 0x06
#define CAN_WRIST_FIRMWARE_UPGRADE 0x07
#define CAN_CLAW_FIRMWARE_UPGRADE 0x08
#define CAN_TARE 0x09
#define CAN_USE_IMU 0x0A
// Runtime tuning (2026-11-17, see common/tuning/tuning.h): the host sets or reads one parameter of one
// joint; the joint's module answers on its *_TUNED id with the value now in effect. Kept in RAM - a
// reboot goes back to the compiled defaults.
#define CAN_TUNE 0x0B

#define USE_IMU_NOT_USE 0
#define USE_IMU_USE 1
#define USE_IMU_AUTO 2

#define CAN_PLATFORM_QUATERNION 0x10
#define CAN_PLATFORM_ACCELEROMETER 0x11
#define CAN_PLATFORM_GYROSCOPE 0x12
#define CAN_PLATFORM_ACCURACY 0x13
#define CAN_PLATFORM_BAROMETER 0x14
#define CAN_PLATFORM_CPU_POWER 0x15
#define CAN_PLATFORM_ENGINES_POWER 0x16
#define CAN_PLATFORM_STATUS 0x17

#define CAN_SHOULDER_QUATERNION 0x20
#define CAN_SHOULDER_ACCELEROMETER 0x21
#define CAN_SHOULDER_GYROSCOPE 0x22
#define CAN_SHOULDER_ACCURACY 0x23
#define CAN_SHOULDER_STATUSES 0x24
#define CAN_SHOULDER_TUNED 0x25
#define CAN_SHOULDER_SETTLE_SPEED 0x26
#define CAN_SHOULDER_FIRMWARE_UPGRADE_CONFIRMED 0x28

#define CAN_ELBOW_QUATERNION 0x30
#define CAN_ELBOW_ACCELEROMETER 0x31
#define CAN_ELBOW_GYROSCOPE 0x32
#define CAN_ELBOW_ACCURACY 0x33
#define CAN_ELBOW_STATUSES 0x34
#define CAN_ELBOW_TUNED 0x35
#define CAN_ELBOW_SETTLE_SPEED 0x36
#define CAN_ELBOW_FIRMWARE_UPGRADE_CONFIRMED 0x38

#define CAN_WRIST_QUATERNION 0x40
#define CAN_WRIST_ACCELEROMETER 0x41
#define CAN_WRIST_GYROSCOPE 0x42
#define CAN_WRIST_ACCURACY 0x43
#define CAN_WRIST_STATUSES 0x44
#define CAN_WRIST_TUNED 0x45
// The wrist's own pitch and side bend since calibration (two floats, deg): the host plans from them
// instead of working them out of the wrist's quaternion a second time (see ArmWrist::getIMUAngles()).
#define CAN_WRIST_ANGLES 0x46
#define CAN_WRIST_FIRMWARE_UPGRADE_CONFIRMED 0x48

#define CAN_CLAW_QUATERNION 0x50
#define CAN_CLAW_ACCELEROMETER 0x51
#define CAN_CLAW_GYROSCOPE 0x52
#define CAN_CLAW_ACCURACY 0x53
#define CAN_CLAW_STATUSES 0x54
#define CAN_CLAW_HEIGHT 0x55
#define CAN_CLAW_RANGE 0x56
#define CAN_CLAW_FIRMWARE_UPGRADE_CONFIRMED 0x58

#define MEMS_SDA_PIN 0
#define MEMS_SCL_PIN 1
#define MEMS_INT_PIN 10
#define WRIST_MEMS_RST_PIN 2
#define MEMS_RX_PIN 0
#define MEMS_TX_PIN 1
#define MEMS_RST_PIN 9
#define MEMS_SCK_PIN 2
#define MEMS_MOSI_PIN 3
#define MEMS_MISO_PIN 0
#define MEMS_CS_PIN 1

#define WITMOTION_XSHUT_PIN 6
#define WITMOTION_INT_PIN 7
#define VL_6180X_SHUT_PIN 13
#define VL_53LOX_SHUT_PIN 14

#define MEMS_UART_ID uart0
#define DETECTORS_SDA_PIN 2
#define DETECTORS_SCL_PIN 3

#define IMU_UPDATE_INTERVAL 5

#define SHOULDER_Z_PIN 11
#define SHOULDER_Y_PIN 7
#define ELBOW_Y_PIN 7
#define WRIST_Z_PIN 9
#define WRIST_Y_PIN 3
#define CLAW_X_PIN 9
#define CLAW_Y_PIN 10
#define CLAW_GRIPPER_PIN 11
#define CAN_RX_PIN 4  
#define CAN_TX_PIN 5

// Onboard WS2812 on the Waveshare RP2040-Zero/Tiny boards.
#define STATUS_LED_PIN 16
#define CAN_SEND_LOOP_TIMEOUT 5
#define CAN_SEND_WAIT_TIMEOUT 50
#define CAN_TX_IN_FLIGHT_TIMEOUT 20
#define CAN_RECEIVE_LOOP_TIMEOUT 50
#define CAN_RECEIVE_WAIT_TIMEOUT 50
#define IMU_WAIT_TIMEOUT 50
#define IMU_NO_DATA_TIMEOUT 500
#define IMU_DEGREE_CHANGE_IGNORE 0.3f

#define ENGINE_TASK_LOOP_TIMEOUT_US 5000

#define CAN_RX_BUFFER_SIZE 64
#define CAN_STATS_LOG_INTERVAL 5
#define CAN_TX_BUFFER_SIZE 16
#define SPEED_BUFFER_SIZE 5

#define RANGE_SHORT_TO_LONG_THRESHOLD 180
#define RANGE_LONG_TO_SHORT_THRESHOLD 150
#define RANGE_INVALID_READING 8191
#define RANGE_LOOP_TIMEOUT 100
#define RANGE_SWITCH_TIMEOUT 50
#define RANGE_SENSOR_STABILIZE_DELAY 10
#define RANGE_SHUTDOWN_DELAY 10
#define RANGE_REINIT_INTERVAL 5000

#define VL53_DEFAULT_ADDR 0x29
#define VL618_DEFAULT_ADDR 0x29

#define VL53_NEW_ADDR 0x2A
#define VL618_NEW_ADDR 0x2B

#define SERVO_MAX_DEGREE_CHANGE 10.0f
#define SERVO_MIN_DEGREE_CHANGE 0.01f

#define SHOULDER_Z_HOME_POSITION 135
#define SHOULDER_Y_HOME_POSITION 90
#define SHOULDER_DEAD_ZONE 0.1f
#define ELBOW_Y_HOME_POSITION 135
#define ELBOW_DEAD_ZONE 0.3f
// Looser than *_DEAD_ZONE: resting accelerometer noise alone spans ~+/-0.5-1 deg.
#define CALIBRATION_SETTLE_TOLERANCE 1.0f
// A calibration stops, holding the servos where they are, once the BNO's fusion has disagreed with
// its accelerometer by more than CALIBRATION_FUSION_MAX_DEG for CALIBRATION_FUSION_MAX_MS, and resets
// the BNO; after CALIBRATION_MAX_FUSION_ABORTS such stops in a row it waits for an engines off/on.
#define CALIBRATION_FUSION_MAX_DEG 10.0f
#define CALIBRATION_FUSION_MAX_MS 300
#define CALIBRATION_MAX_FUSION_ABORTS 3
#define CALIBRATION_STABLE_TIME_MS 500
#define SHOULDER_Y_RECOVERY_TIME_MS 4000
#define WRIST_Z_HOME_POSITION 135
#define WRIST_Y_HOME_POSITION 90
#define WRIST_DEAD_ZONE 0.3f
#define WRIST_Y_RECOVERY_TIME_MS 4000
// PWM each servo holds at gravity-verified vertical (gravity reading 90), from live calibrations
// (2026-08-25..10-31: elbow 138-146, wrist 134-140). Only the boot-time default for
// elbowYHomeAngle/wristYHomeAngle - every successful calibration overwrites it with where it landed.
#define ELBOW_Y_PWM_AT_VERTICAL 142.0f
#define WRIST_Y_PWM_AT_VERTICAL 137.0f
#define CLAW_X_HOME_POSITION 135
#define CLAW_Y_HOME_POSITION 90
// Gripper travel measured live 2026-10-31 (command -> jaw gap): 175 closed and straining (servo
// hums), 174 closed without load, 172 ~1mm, 160 ~20mm, 150 ~32mm, 140 ~46mm, 130 ~57mm,
// 120 ~67mm, 110 ~74mm, 100 ~80mm, 95 ~83mm, 90 ~83.5mm, 85 ~84mm = the mechanical open stop (no
// hum yet). OPEN keeps 5deg off that stop for ~the same opening; SAFE_MAX keeps 175 so it can
// squeeze an object, nothing past it.
#define CLAW_GRIPPER_CLOSED 174.0f
#define CLAW_GRIPPER_OPEN 90.0f
#define CLAW_GRIPPER_SAFE_MIN 85.0f
#define CLAW_GRIPPER_SAFE_MAX 175.0f
// Was 30, far outside the travel. Only the servo's starting physicalAngle - open-loop, so the first
// commanded move after power-up jumps here first; mid-travel keeps that jump smallest.
#define CLAW_GRIPPER_HOME_POSITION 130
// Open-loop axes (self-referential imuAngle: the gripper always, X/Y in manual mode) - the dead zone
// is pure stopping precision there (the old shared 2deg claw dead zone stopped every move 2deg short).
#define CLAW_OPEN_LOOP_DEAD_ZONE 0.3f
// Claw mapping, measured live 2026-10-31 (manual mode). clawY: PWM 178 = horizontal, ~90 = vertical,
// 1:1. That session found a stop ~14deg past vertical (~PWM 81) and kept commands >= 85; the user
// says the working range is the servo's whole 0-180 - the mechanical limit is well past it
// (2026-11-06) - so the claw bends +-90deg either way from in line with the wrist. Unpowered, the claw is light
// enough to stay at whatever angle it was left - nothing can be assumed about where it is.
// clawX (roll of the jaws, 1:1): jaws level at PWM 130, where its gravity roll reads ~64.5deg with
// clawY horizontal (the IMU is fixed to the jaws; the user remounted them as level as mechanically
// possible). Only boot defaults for clawYHomeAngle/clawXHomeAngle - calibration overwrites them.
#define CLAW_Y_PWM_AT_VERTICAL 90.0f
#define CLAW_Y_SAFE_MIN 0.0f
#define CLAW_Y_PWM_HORIZONTAL 178.0f
#define CLAW_X_PWM_AT_LEVEL 130.0f
#define CLAW_X_ROLL_AT_LEVEL 64.5f
// Calibration dead zone for X/Y (IMU-closed): must stay under CALIBRATION_SETTLE_TOLERANCE, or the
// settle check can never pass (the old shared 2deg claw dead zone was twice that tolerance).
#define CLAW_CALIBRATION_DEAD_ZONE 0.5f

#define SERVO_DEAD_ZONE 0.3f
// Hold hysteresis of the IMU-held joints (Servo::setHoldHysteresis()): once settled, an error has to
// stay above this long before the joint is corrected again (2026-11-06, knock-on-the-table hunting).
#define HOLD_WAKE_DEG 1.0f
#define HOLD_WAKE_MS 400
#define HOLD_WAKE_NOW_DEG 2.5f
// Streaming (Servo::streamTo()): a host command with 0 < timeMS <= STREAM_MAX_MS is a waypoint of a
// stream (a straight-line move sends one every ~100 ms), not a move of its own. STREAM_END_MS after
// the last one is reached the joint goes back to its hold. STREAM_GAIN (1/s) pulls the command after
// the IMU's distance to the setpoint, low-passed over STREAM_ERROR_TAU_SEC, on top of the setpoint's
// own speed. Measured live (2026-11-07, servo traces of a 4 s host move): at 3/s on the raw error this
// term swung the motors back and forth - the link slips through its backlash ahead of the setpoint and
// the correction yanks the motor back, pumping a 2-4 Hz swing (elbow 1.6deg rms); at 0.5/s the swing
// fell 2.5x but the sag under the growing load wasn't taken up (the tip stopped 12 mm low, then the
// hold crept it up for 2-3 s). Low-passed, 2/s acts like ~0.4/s on the swing and takes up the sag in
// ~0.5 s (the loop: zeta ~0.65).
#define STREAM_MAX_MS 250
// A stream's setpoint starts on the host's first waypoint, and the gap from where the link is fades
// out over STREAM_ENTRY_SEC - longer for a big gap, at most STREAM_ENTRY_MAX_SPEED_DEG_PER_SEC
// (Servo::streamTo()). Measured live (2026-11-17): the first waypoint was 0.5-0.8deg off each module's
// own IMU reading (the host's model vs the module's), the setpoint crossed that in 60 ms, and at
// vertical the kick tipped the whole arm through its backlash - the tip 46 mm forward and back.
#define STREAM_ENTRY_SEC 0.5f
#define STREAM_ENTRY_MAX_SPEED_DEG_PER_SEC 5.0f
#define STREAM_END_MS 300
#define STREAM_GAIN 2.0f
#define STREAM_ERROR_TAU_SEC 0.3f
// After a stream the hold corrects at no more than STREAM_SETTLE_SPEED_DEG_PER_SEC until it settles:
// the motor may have backlash to cross before it reaches the link, and at the hold's own speed it
// yanked the link back when it got there (2026-11-10). A setpoint within STREAM_STILL_DEG of where its
// link started is a joint the plan doesn't move (Servo::setStreamCorrectWhenStill()).
#define STREAM_STILL_DEG 1.0f
// End-of-stream overshoot shift (Servo::streamTick(), learned by the host): blended in once the
// setpoint is STREAM_END_ARM_DEG on its way, has peaked at STREAM_END_MIN_PEAK_DEG_PER_SEC or more and
// slows from that peak (smoothed over STREAM_END_SMOOTH_SEC).
#define STREAM_END_MIN_PEAK_DEG_PER_SEC 2.0f
#define STREAM_END_ARM_DEG 2.0f
#define STREAM_END_SMOOTH_SEC 0.1f
#define STREAM_END_SPEED_TAU_SEC 0.15f // the setpoint speed the end weight is taken from (low-passed)
// A stream that ends this close to its target leaves it to the hold's hysteresis (Servo::streamTick()).
#define STREAM_SETTLE_TOLERANCE_DEG 0.7f
#define STREAM_SETTLE_SPEED_DEG_PER_SEC 1.0f
// What a stream leaves of the IMU error its command takes out in one minimum-jerk move of at least
// STREAM_SETTLE_PLAN_MS, peaking at no more than STREAM_SETTLE_PLAN_SPEED_DEG_PER_SEC
// (Servo::streamTick()); the hold after it is the gentle one above. At 10deg/s the elbow, 6.4deg
// short at the top of a move back to vertical, swung 4deg past and 9 back, the wrist 15 (tip -109 mm)
// - by vertical the backlash isn't loaded (2026-11-12); 4deg/s makes that a 3 s settle, a 1.5deg one
// still 1 s.
#define STREAM_SETTLE_PLAN_MS 1000
#define STREAM_SETTLE_PLAN_SPEED_DEG_PER_SEC 4.0f
// Before it settles, a stream's end waits for the link to calm (Servo::streamWaitTick()): the command
// held, the link's IMU taken in blocks of STREAM_CALM_BLOCK_MS; a block spanning no more than
// STREAM_CALM_SPAN_DEG is calm and its mean error is what's left. Still swinging after
// STREAM_CALM_WAIT_MAX_MS, the mean of the last STREAM_CALM_MEAN_BLOCKS blocks (~one swing) is.
// Measured live (2026-11-17, two moves back up to a 1deg lean): at the top the arm swings with a ~2 s
// period, and the settle took a snapshot of it - once the elbow 3.5deg and the wrist 3.3 behind on the
// way back, all three joints settled at once while the arm was swinging back anyway, and the tip went
// from +38 to -36 mm.
#define STREAM_CALM_BLOCK_MS 500
#define STREAM_CALM_SPAN_DEG 0.6f
#define STREAM_CALM_WAIT_MAX_MS 3000
#define STREAM_CALM_MEAN_BLOCKS 4
// A child joint settles only once its parents have (Servo::setAncestorsSettling()): measured live
// (2026-11-17), the elbow settling 0.8-1.3deg after a move took the wrist's absolute angle with it,
// and the wrist's hold woke 0.4 s later and crept after it, jumping 1-1.5deg (the shoulder's settle
// did the same to the elbow). Waiting on a parent stuck settling ends after STREAM_PARENT_WAIT_MAX_MS.
#define STREAM_PARENT_WAIT_MAX_MS 8000
// While a joint settles after a stream it sends its command's speed (CAN_*_SETTLE_SPEED, float deg/s,
// every engine tick, then one 0), and its child turns against it while it waits (see
// Servo::setParentSettleSpeed()). Measured live (2026-11-17): the wrist waited with its command still
// while the shoulder settled 2.2deg and the elbow 1.75, the claw went with them - the tip 40 mm away -
// and the wrist's own settle brought it back. A speed older than SETTLE_SPEED_STALE_MS is 0.
#define SETTLE_SPEED_STALE_MS 50
// The most the stream correction may add to a joint's command over one stream (Servo::streamTick()):
// a real sag took up to ~6deg at the shoulder and over 8 at the elbow lifting the forearm (2026-11-11).
#define STREAM_CORRECTION_MAX_DEG 12.0f
// The stream correction trusts the IMU only while its quaternion agrees with its own accelerometer
// (LocalBNO::getFusionDriftDeg()): past this it holds still, the plan's feed-forward carries on, and it
// resumes once they agree again (Servo::setFusionError()). A diverging BNO used to get the whole
// STREAM_CORRECTION_MAX_DEG: the shoulder's (2026-11-05, 14deg off on a 10deg move) swept the arm, the
// elbow's (2026-11-17) drove it 12deg past the plan, while the BNO's own reset waits for 15deg over 1 s
// (localBNO.cpp). The real corrections are as big as the bound - measured live (2026-11-19, five host
// moves from the park pose) up to 4.5deg at the shoulder, 7.4 at the elbow, 5.6 at the wrist - so the
// bound can't be what stops a bad IMU. A pause by mistake costs only the correction meanwhile (a test
// with the wrist's at 2deg: paused 1.5 s of a 4 s move, the link ended 2.0deg past instead of 1.1).
// The first version paused on LocalBNO::getFusionErrorDeg(), which grows with the joint's speed: the
// wrist's 12deg tripped on a plain 4 s move (see getFusionDriftDeg()).
#define STREAM_FUSION_GATE_DEG 6.0f
#define WRIST_STREAM_FUSION_GATE_DEG 12.0f
// Dither (Servo::setDither()): ramped in and out over DITHER_RAMP_SEC, kept DITHER_HOLD_MS after the
// joint stops moving; a stream setpoint slower than DITHER_MIN_SPEED_DEG_PER_SEC isn't motion.
#define DITHER_RAMP_SEC 0.1f
#define DITHER_HOLD_MS 100
#define DITHER_MIN_SPEED_DEG_PER_SEC 0.5f
// Active damping of the IMU-held pitch joints (Servo::setDamping(), see ArmShoulder's for the
// measurements): the link-rate low-pass, the deadband below which a rate doesn't close the loop, and
// a rate that starts a servo trace.
// Not faster or stronger on the elbow and wrist: with 30 ms and 0.08 s (instead of 60 ms, 0.05 s) a 4 s
// host move ended in a violent swing of the whole arm at the work pose - links at 120-190deg/s, the
// damping offsets pinned at +-3deg - until the user cut the engines (2026-11-12). At rest by
// vertical it had looked fine.
// 0.15 s since 2026-11-17: braking a 4 s move down the whole arm shook at ~3.5 Hz (elbow and wrist
// +-1.5deg, their link speed 20-35deg/s off the setpoint's), and the damping filtered over 60 ms came
// 20-40 ms late on top of the servo's own lag - it pumped that shake. Undamped, the braking shake
// halved but a slower ~1.5 Hz sway grew while accelerating; over 0.15 s the filter passes that sway
// and not the shake: braking 0.34 instead of 1.02deg rms at the wrist, accelerating 0.28-0.57 instead
// of 0.62 (tuned over CAN, two moves each way).
#define DAMPING_RATE_TAU_SEC 0.15f
#define DAMPING_DEADBAND_DEG_PER_SEC 3.0f
#define DAMPING_TRACE_RATE_DEG_PER_SEC 20.0f
// Break-away lead (see Servo::setBreakawayLead()). Measured live (2026-11-19, elbow traces of 5 s
// park <-> down moves): lifting to the parking pose the elbow's command went 6.4-7.8deg before the
// link moved at all - 1.5 s of the 5 - while the correction, low-passed, took that up; when the link
// broke away it raced 4-6deg past the setpoint. Lowering it moved after 1-2deg. So as the setpoint
// sets out the command leads the plan by what the link needed last time that way: the setpoint this
// far from its start shows the direction, the lead fades in over STREAM_LEAD_FADE_SEC, and the link
// this far along has broken away - how far the plan had got ahead by then (plus the correction, less
// any run past the setpoint just after) is what to add, at STREAM_LEAD_LEARN_RATE. It fades out with
// the end shift's weight as the setpoint brakes: kept to the end (same day), the link, lifting, trailed
// its command by a steady ~5deg until the plan slowed, then caught up within a second and ran 4-5deg
// past - braking takes less motor torque than driving (the gears' reflected motor inertia), and lowering
// it had trailed by ~0.5 only.
#define STREAM_LEAD_ARM_DEG 0.3f
#define STREAM_LEAD_FADE_SEC 0.3f
#define STREAM_LEAD_BREAK_DEG 1.0f
#define STREAM_LEAD_AHEAD_SEC 0.5f
#define STREAM_LEAD_LEARN_RATE 0.3f
#define STREAM_LEAD_LEARN_MIN_TRAVEL_DEG 4.0f  // the shoulder's 5deg moves to the parking pose too
#define STREAM_LEAD_MAX_DEG 9.0f
// The lead scales with the setpoint's own speed, low-passed over STREAM_LEAD_SPEED_TAU_SEC: all of it
// at STREAM_LEAD_FULL_SPEED_DEG_PER_SEC and up. Measured live (2026-11-21, a 4 s move to the parking
// pose after elbow-only moves): the host's last correction moved the shoulder 0.3deg and the elbow
// 1.4deg, slowly, and each took its whole learned lead (3 / 4.4deg) - more than the move itself: the
// elbow jumped 6deg in 0.5 s, the shoulder, upright, fell 5.3deg through its backlash and the wrist
// swung at 20deg/s. On the elbow's servo model (identified 2026-11-20) this scale takes such a 1.4deg
// correction's run past from 5.4 to 1.4deg and a small way back from 2.7 to none, and leaves the 5 s
// lifts and 6deg shoulder-like moves as they were. Low-passed: the host's waypoints come in 0.1deg
// steps, so a slow setpoint's speed comes in pulses.
#define STREAM_LEAD_FULL_SPEED_DEG_PER_SEC 2.0f
#define STREAM_LEAD_SPEED_TAU_SEC 0.3f
// The values to start from, below what it took. Lifting is toward smaller angles while the arm leans
// to the side growing commands tilt it to - where it works; the side of vertical a joint's own link is
// on doesn't decide it (a shoulder just past vertical still lifts an arm leaning out).
#define ELBOW_LEAD_TOWARD_SMALLER_DEG 4.0f
#define ELBOW_LEAD_TOWARD_LARGER_DEG 0.5f
// The shoulder (same traces, the move back to the parking pose 92 -> 86): its command led 5.6-6.3deg
// before it moved - 2.4 s of 4 - and the arm then dropped 4deg in 0.4 s, which the elbow, holding the
// forearm's tilt, could only take as a 4deg run past its own setpoint. Started lower than the elbow:
// it swings the whole arm when it lets go.
#define SHOULDER_LEAD_TOWARD_SMALLER_DEG 3.0f
#define SHOULDER_LEAD_TOWARD_LARGER_DEG 0.5f

#define FLASH_EEPROM_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define FLASH_EEPROM_SIZE FLASH_SECTOR_SIZE
#define HOME_POSITIONS_EEPROM_OFFSET 0
#define FLASH_WAIT_TIMEOUT 5000 // 5 seconds in milliseconds

#define FLASH_SETTINGS_NOTIFICATION_INDEX 1
#define FLASH_BUFFER_SIZE 64

#define PARAMETER_IS_NAN 65535

#define ARM_PLATFORM 1
#define ARM_SHOULDER 2
#define ARM_ELBOW 4
#define ARM_WRIST 8
#define ARM_CLAW 16

#define ARM_POSITION_OK 1
#define ARM_ENGINE_OK 2
#define BUS_RECEIVING_OK 4
#define ARM_X_CALIBRATING 8
#define ARM_Y_CALIBRATING 16
#define ARM_Z_CALIBRATING 32
#define ARM_UPGRADING 64
#define ARM_TARE_ERROR 128
#define ARM_CALIBRATED 256
#define ARM_USE_IMU_ENABLED 512
#define ARM_USE_IMU_AUTO 1024
// The joint's link is calming after a stream, its command held (Servo::streamWaitTick()) - the host
// averages the joint angles over it to learn the end-of-move overshoot (2026-11-17). Shoulder: Y,
// elbow: Y, wrist: Y.
#define ARM_STREAM_WAITING 2048
// The joint is still putting its last stream right - calming or settling (Servo::isStreamSettling()).
// Its children wait for it before they settle (Servo::setAncestorsSettling()). Shoulder: Y, elbow: Y,
// wrist: Y.
#define ARM_SETTLING 4096
// The joint's IMU was reset with the engines on and it holds its command while it re-anchors in place
// (ArmElbow::recoverInPlace()) - the host stops the running move.
#define ARM_IMU_RECOVERING 8192
// The status frame's upper half carries the joint's calibrated home commands - the command that holds
// the calibration pose (2026-11-06, for the kinematics on the host's clients, see
// ArmPart::setHomeAngles()): bits 32-47 the first, 48-63 the second, 0.01deg each, 0xFFFF = none.
// Shoulder: Y, and Z's command now - Z runs open-loop, so that's its angle (its home is always
// SHOULDER_Z_HOME_POSITION), and its IMU yaw drifts several degrees in minutes. Elbow: Y, none.
// Wrist: Y, Z. Claw: Y, and Y's command now - it runs open-loop, so that's its angle; its IMU only
// estimates it (2 deg off on 2026-11-06, which jumped the claw at the start of a line).
// Valid while ARM_CALIBRATED is set.
#define STATUS_HOME_SHIFT_FIRST 32
#define STATUS_HOME_SHIFT_SECOND 48
#define STATUS_HOME_NONE 0xFFFF

#define ERROR_ENGINE_TASK_CREATION_FAILED -1
#define ERROR_PART_IS_NOT_CALIBRATED -2



// Register Bits{{
#define BIT31 0x80000000
#define BIT30 0x40000000
#define BIT29 0x20000000
#define BIT28 0x10000000
#define BIT27 0x08000000
#define BIT26 0x04000000
#define BIT25 0x02000000
#define BIT24 0x01000000
#define BIT23 0x00800000
#define BIT22 0x00400000
#define BIT21 0x00200000
#define BIT20 0x00100000
#define BIT19 0x00080000
#define BIT18 0x00040000
#define BIT17 0x00020000
#define BIT16 0x00010000
#define BIT15 0x00008000
#define BIT14 0x00004000
#define BIT13 0x00002000
#define BIT12 0x00001000
#define BIT11 0x00000800
#define BIT10 0x00000400
#define BIT9 0x00000200
#define BIT8 0x00000100
#define BIT7 0x00000080
#define BIT6 0x00000040
#define BIT5 0x00000020
#define BIT4 0x00000010
#define BIT3 0x00000008
#define BIT2 0x00000004
#define BIT1 0x00000002
#define BIT0 0x00000001

#define CAN_EVENT_RX BIT0
#define CAN_EVENT_TX BIT1
#define CAN_EVENT_REINIT BIT2

#define PLATFORM_STATUS_CAN_SEND_OK 1
#define PLATFORM_STATUS_SHOULDER_QUATERNION_OK 2
#define PLATFORM_STATUS_ELBOW_QUATERNION_OK 4
#define PLATFORM_STATUS_WRIST_QUATERNION_OK 8
#define PLATFORM_STATUS_CLAW_QUATERNION_OK 16
#define PLATFORM_STATUS_CLAW_RANGE_OK 32
#define PLATFORM_STATUS_ENGINES_ON 64
#define PLATFORM_STATUS_CAMERA_ON 128
#define PLATFORM_STATUS_DETECTORS_ON 256
#define PLATFORM_STATUS_CPU_POWER_ON 512

#define LOG_QUEUE_MSG_SIZE 128
#define LOG_QUEUE_MESSAGES 32