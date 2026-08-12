# thinker-robot-engine-controller — operational notes

Firmware for the RP2040 joint modules (shoulder, elbow, wrist, claw) of the robotic arm. Host platform (ESP32) is `../thinker-robot-host`, reachable at `http://192.168.1.120`.

## Identifying which board is which

`ttyACM0`/`ttyACM1` swap unpredictably across power cycles and reflashes. Never assume which is which — always check `/dev/serial/by-id/` and match by Pico serial number:

```bash
ls -la /dev/serial/by-id/
```

- `E66250758B3D6B21` → shoulder
- `E66250758B484121` → elbow

(Wrist/claw serials not yet recorded here — add them if you flash those boards.)

## Building and flashing

```bash
cmake -S . -B build
cmake --build build
```

Then deploy with the matching script (each one builds, disables engines via the host `/set` API, reboots the target board into BOOTSEL, copies the `.uf2`, and lets it reboot into the new firmware):

```bash
./deploy-shoulder.sh
./deploy-elbow.sh
./deploy-wrist.sh
./deploy-claw.sh
```

## Power control (via the host's `/set` HTTP endpoint)

```bash
curl -X POST http://192.168.1.120/set -H "Content-Type: application/json" -d '{"enginesEnabled": true}'
```

Fields (all optional booleans, only send the ones you want to change):
- `enginesEnabled` — servo motor power. This is a host-side GPIO cut only — it does **not** power down the RP2040 logic/USB, and the modules have no way to detect it happened except by losing `platform.isPositionOK()`. With engines off, unpowered joints droop under gravity (most visible on shoulder Y).
- `cpuPowerEnabled` — CPU power rail.
- `cameraEnabled` — camera power.
- `detectorsPowerDisabled` — note the inverted name: `true` disables detector power, `false` enables it.

`useIMU` is a separate global (all-modules) field, also sent via `/set`:
```bash
curl -X POST http://192.168.1.120/set -H "Content-Type: application/json" -d '{"useIMU": "use"}'
```
Values: `"not-use"` (self-referential/physically-inert — the emergency fallback, not the normal state), `"use"` (the intended default), `"auto"` (currently == `"use"`). Broadcasts to every module over CAN — toggling it always affects both shoulder and elbow together, there's no per-module version. Both shoulder and elbow default to `use`.

## Reading live logs

Each module streams debug logs over its USB CDC serial port at 115200 baud:
```bash
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0
```
Only run one reader per port at a time — two concurrent `cat` processes on the same device split the incoming bytes and corrupt both streams.

## Live status / WebSocket

The host broadcasts full arm status (quaternions, accelerometers, per-module status bitfields) over `ws://192.168.1.120/ws`, decoded via `../imuVisualizer/static/modules/schema.js`. See scratch decoder scripts built during past debugging sessions for the schema shape (status bitfield layout, quaternion/accelerometer field names).

## Safety notes

- Always confirm with a human before enabling engines on the physical arm — droop-then-recovery and calibration motion are real, visible movements.
- After any engine power cycle, both shoulder and elbow recalibrate automatically; shoulder's calibration is gated by a settle check before it reports itself calibrated (elbow's own calibration waits on that).
- `useIMU: use` on elbow is now the default (as of 2026-08-11 night) — earlier incidents (2026-08-10) traced to root causes since fixed (see project memory `robot_arm_session_status` for the full list). Still confirm with a human before enabling engines regardless.
