# Tremor Suppression Glove — Implementation Plan

## 1. System Summary

### What We Are Building

A closed-loop vibrotactile tremor suppression glove for Parkinson's disease patients. The glove detects pathological hand tremor (4–6 Hz) using an IMU and responds with amplitude-modulated vibration through three LRA actuators on the wrist, delivered at intensity proportional to tremor severity via a PI controller.

The full sensing-to-actuation pipeline runs at 200 Hz on an STM32F401CCU6:

```
MPU-6050 IMU (I²C1, 400 kHz)
  → 14-byte burst read (accel XYZ + gyro XYZ)
  → complementary filter → stable wrist angle
  → gyro vector magnitude = sqrt(ωx² + ωy² + ωz²)
  → IIR biquad bandpass (3–8 Hz, 4th-order Butterworth cascade)
  → running RMS over 200 ms ring buffer
  → state machine (IDLE/ACTIVE) with hysteresis
  → PI controller (Kp configurable, Ki=0 default)
  → DriveCommand (intensity byte per actuator, 0–127)
  → TCA9548A I²C mux (I²C2, PB10/PB11)
  → 3× DRV2605L haptic drivers (RTP mode)
  → 3× LRA actuators (volar wrist 100%, volar forearm 70%, dorsal wrist 50%)
```

Telemetry is transmitted at 20 Hz over UART1 DMA → HM-10 BLE → browser Web Bluetooth API → Chart.js live graph.

### Key Design Constraints

| Constraint | Value | Source |
|---|---|---|
| Control loop rate | 200 Hz (5 ms budget) | TDR 2.2 |
| End-to-end actuation latency | < 5 ms | TDR 2.2 |
| Tremor detection latency | < 500 ms from onset | TDR 2.2 |
| False positive rate | < 10% | TDR 2.2 |
| RMS amplitude reduction | > 30% | TDR 2.2 |
| BLE telemetry rate | 20 Hz | TDR 2.2 |
| I²C bus 1 (IMU) | PB6/PB7, 400 kHz | TDR 4.3 |
| I²C bus 2 (haptic chain) | PB10/PB11, 400 kHz | TDR 4.3 |
| UART1 (BLE) | PA9/PA10, 9600 baud | Brief |
| No HAL_Delay() anywhere | — | Brief |
| No malloc in firmware | — | Brief |
| All waits via flags/callbacks | — | Brief |
| Algorithm modules must compile on native (host PC) | — | Brief |

### Riskiest Parts

1. **I²C timing budget is tight.** Three DRV2605L writes through the TCA9548A mux take ~1.2 ms each cycle at 400 kHz. If IMU read + three haptic writes + filter computation exceeds 5 ms, the loop overruns. Measured budget in the design notes is ~2.5 ms — this is fine but leaves no room for blocking I²C errors. Every I²C call must be non-blocking or DMA-driven, with stale-data fallback.

2. **DRV2605L initialisation is fragile.** The chip must be written to in a specific sequence (MODE → FEEDBACK_CTRL → leave RTP mode ready) before RTP writes will do anything. If one channel fails during init, it must be flagged and skipped without stopping the others.

3. **Ring buffer fill latency.** The RMS window is 200 ms (40 samples at 200 Hz). For the first 200 ms of operation the buffer is partially filled. Incorrect RMS on partial data could cause a false-positive tremor detection immediately after boot, triggering actuators before calibration completes.

4. **State machine timestamp discipline.** The brief specifies timestamps in milliseconds, not sample counts. SysTick wraps at 2^32 ms (~49 days) — not a real concern — but `uint32_t` subtraction handles it correctly regardless.

5. **BLE packet parsing in browser.** The HM-10 is a transparent UART bridge; BLE notifications can fragment or merge bytes depending on MTU. The JS parser must re-sync on the 0xAA header byte and validate checksum rather than trusting packet boundaries.

6. **Web Bluetooth API** only works in Chrome/Edge over HTTPS or localhost. This is a demo environment constraint that must be documented.

7. **MockTransport must be realistic enough to exercise the parser.** If it produces data that never fails the checksum, parser bugs (off-by-one in byte offsets, endianness) won't be caught until real hardware is connected.

---

## 2. Answers to Pre-Implementation Questions

### Q1: Wrist angle from accelerometer for the complementary filter — which axis?

The complementary filter fuses gyro (accurate short-term, drifts) with accelerometer (accurate long-term, noisy). For wrist flexion-extension tremor:

- **Gyro contribution**: integrate ωy (pitch axis when IMU is mounted dorsal wrist, Y-axis aligned with the wrist's flexion-extension rotation axis). The raw gyro rate in °/s is converted to rad/s and multiplied by dt.
- **Accelerometer contribution**: `accel_angle = atan2f(ay, az)` — the pitch angle derived from gravity projection. This is gravity-referenced and works as the long-term anchor for the complementary filter.

```c
// At 200 Hz, dt = 0.005f seconds
angle = ALPHA * (angle + gyro_y_rads * dt) + (1.0f - ALPHA) * atan2f(ay, az);
```

`ALPHA = 0.96f` per the software notes. The actual tremor detection input is **gyro vector magnitude** `sqrt(ωx²+ωy²+ωz²)` — the complementary filter's angle output is used for the wrist angle telemetry field, not directly for detection. Both are computed each cycle.

**Important**: the CMSIS filter input is the gyro vector magnitude (scalar), not the angle. This is a 1D signal fed into the biquad bandpass.

### Q2: DRV2605L init — boot-time or lazy? What if a channel fails?

**Boot-time for all three channels**, done sequentially during the `hal_haptic_init()` call before entering the main loop.

Rationale: lazy init would add per-cycle I²C overhead on the first write to each channel, which could cause a loop overrun at an unpredictable time. Boot init is predictable and happens before the 200 Hz loop starts.

**If a channel fails to init**: flag it in a `uint8_t haptic_channel_ok[3]` bitmask, log the error count in telemetry (`error_flags` field), and skip that channel in `haptic_set_drive()` for the rest of the session. Do not attempt re-init during the control loop. The other two channels continue to function. The system must recover from a single I²C channel failure without power cycling — this is a non-functional requirement (TDR 2.2).

### Q3: What should the mock IMU generate?

A **realistic composite signal** with three components:
1. **Tremor component**: 5 Hz sine wave, configurable amplitude (default enough to exceed ON_THRESHOLD after the RMS window fills)
2. **Voluntary movement component**: 0.8 Hz sine wave at 3× the tremor amplitude (simulates reaching/holding)
3. **Sensor noise**: Gaussian noise, σ ≈ 0.02 rad/s

```
gyro_y(t) = A_tremor * sin(2π * 5 * t) 
           + A_voluntary * sin(2π * 0.8 * t)
           + noise(σ=0.02)
```

This exercises the bandpass filter's job of extracting the 5 Hz component and rejecting the 0.8 Hz voluntary movement. A pure sine wave at 5 Hz would test the filter but not discriminate it from voluntary motion. The composite is essential for testing the state machine's false-positive rate.

The mock should also have a **tremor-off period** (e.g., seconds 5–10 of a 20-second cycle) where only the voluntary movement component is present, verifying the ACTIVE→IDLE transition.

### Q4: Ring buffer during fill (first 200 ms)

The `RingBuf` struct carries a `count` field that tracks how many samples have been pushed, capped at the buffer capacity:

```c
typedef struct {
    float   buf[RMS_WINDOW_SAMPLES];  // 40 samples at 200 Hz
    uint16_t head;
    uint16_t count;                   // valid samples, max = RMS_WINDOW_SAMPLES
} RingBuf;
```

`rms_compute()` uses `rb->count` not the full window size. During the first 200 ms, the RMS is computed over fewer samples — the value is lower than steady-state, so it will not trigger the ON_THRESHOLD falsely (it converges upward to the true RMS as the window fills).

The 2-second calibration routine at boot means the ring buffer is fully populated before calibration completes, so by the time the state machine becomes active, `count == RMS_WINDOW_SAMPLES`.

### Q5: FileTransport packet rate handling

- **Fewer than 20 packets/second in the file**: deliver packets at their actual rate from the file's timestamps. Do not inject synthetic packets. The chart will display gaps as flat lines — this is correct behavior and tells the user the file had sparse data.
- **More than 20 packets/second in the file**: throttle delivery to exactly 20 Hz using a 50 ms interval timer. Skip packets that arrive within the 50 ms window to maintain the 20 Hz rate. Do not buffer and batch — that would cause timestamp jitter.

If the file has no timestamps (raw binary dump), deliver at a fixed 20 Hz regardless.

---

## 3. Module Build Order

Build order is driven by two rules: (a) test immediately on the native platform, (b) each module depends only on modules below it.

```
Layer 0: Ring buffer (no dependencies, pure data structure)
Layer 1: Filters module (depends on ring buffer for RMS)
Layer 2: Tremor detector (depends on filters)
Layer 3: PI controller (depends on tremor detector for tremor_active flag)
Layer 4: Telemetry serialiser (depends on all above for packet fields)
Layer 5: HAL interface definitions + mock implementations (enables integration tests)
Layer 6: Concrete HAL drivers (IMU, haptic, BLE) — target only, not native
Layer 7: main.c integration
Layer 8: Web app (MockTransport → FileTransport → BleTransport)
```

---

## 4. Testing Strategy Per Module

### 4.1 Ring Buffer (`firmware/src/ring_buffer.c`)

| Test | Input | Expected output |
|---|---|---|
| Push fills to capacity | Push N+1 values into N-size buffer | Head wraps, oldest value overwritten |
| Count caps at capacity | Push N+2 into N-size buffer | `count == N` |
| RMS on empty | count=0 | Returns 0.0f |
| RMS on partial fill | Push 5 values into 40-size buffer | `count == 5`, RMS computed over 5 values |
| RMS on full | Push 40 known values | RMS matches known answer (e.g., all 1.0 → RMS=1.0) |
| Rolling update correctness | Shift one sample | RMS changes correctly |

### 4.2 Filters (`firmware/src/filters.c`)

Run `generate_coeffs.py` first to get the biquad coefficients. Tests run on the **native** platform.

| Test | Input | Expected output |
|---|---|---|
| Bandpass passes 5 Hz | Sine wave at 5 Hz, 200 Hz sample rate | Output amplitude ≥ 90% of input |
| Bandpass rejects 1 Hz | Sine wave at 1 Hz | Output amplitude ≤ 5% of input |
| Bandpass rejects 15 Hz | Sine wave at 15 Hz | Output amplitude ≤ 5% of input |
| Complementary filter DC drift rejection | Static tilt + integrated gyro drift | Angle error < 2° after 10 s |
| Complementary filter tracks step input | Step change in gyro | Angle convergence within 0.5 s |
| Filter state resets correctly | `filter_reset()` then 5 Hz input | No transient from previous state |

Tolerance: frequency response tests accept ±1 dB at the test frequency; stopband tests accept −20 dB minimum.

### 4.3 Tremor Detector (`firmware/src/tremor.c`)

| Test | Input | Expected output |
|---|---|---|
| IDLE→ACTIVE transition | RMS > ON_THRESHOLD for 300 ms | State becomes ACTIVE after 300 ms |
| IDLE stays IDLE on brief spike | RMS > ON_THRESHOLD for 100 ms, then drops | State stays IDLE |
| ACTIVE→IDLE transition | RMS < OFF_THRESHOLD for 500 ms | State becomes IDLE after 500 ms |
| ACTIVE holds through brief drop | RMS < OFF_THRESHOLD for 200 ms | State stays ACTIVE (hold-off not elapsed) |
| Thresholds are runtime-configurable | Set ON=0.1, OFF=0.05 | Transitions occur at new values |
| Timestamps not sample counts | Drive at 100 Hz instead of 200 Hz | 300 ms / 500 ms transition times unchanged |

### 4.4 PI Controller (`firmware/src/control.c`)

| Test | Input | Expected output |
|---|---|---|
| Zero output when tremor_active=false | Any amplitude, tremor_active=false | DriveCommand all zeros |
| Proportional output scaling | Kp=1.0, amplitude=64 | drive ≈ 64 (within ±1 LSB) |
| Output clamps at 127 | Kp=10.0, large amplitude | drive == 127 |
| Output clamps at 0 | Negative error (shouldn't occur, but) | drive == 0 |
| Integral accumulation | Ki=0.5, sustained error | drive increases over time |
| Anti-windup: integral clamps independently | Ki large, actuator saturated | I_term ≤ I_MAX, no runaway |
| Ki=0 default behaves as P-only | Ki=0.0 | drive = clamp(Kp * amplitude, 0, 127) |
| Per-actuator weights applied | Amplitude=100, weights [1.0, 0.7, 0.5] | ch0=drive, ch1=0.7*drive, ch2=0.5*drive |

### 4.5 Telemetry (`firmware/src/telemetry.c`)

| Test | Input | Expected output |
|---|---|---|
| Packet serialisation | Known field values | Byte-exact expected buffer |
| Header byte = 0xAA | Any fields | `buf[0] == 0xAA` |
| Checksum = XOR of bytes [1..N-2] | Known buffer | Checksum byte matches manual computation |
| `telemetry_validate` accepts valid | Valid packet | Returns true |
| `telemetry_validate` rejects bad checksum | Flip one byte | Returns false |
| `telemetry_validate` rejects wrong header | buf[0] = 0x00 | Returns false |
| Struct is __packed / no padding | `sizeof(TelemetryPacket)` | Equals sum of field sizes exactly |
| Timestamp wraparound | timestamp_ms = 0xFFFF → 0x0000 | No assertion, treated as valid |

### 4.6 HAL Mock Implementations (native testing)

The mock HAL implements the same function signatures as the real HAL but operates in memory:

- `hal_imu_mock`: generates composite tremor signal (Q3 above), configurable via `mock_imu_set_tremor(amp, freq_hz, on)`
- `hal_haptic_mock`: records last intensity written per channel in a `uint8_t last_drive[3]` array; provides `mock_haptic_get_drive(ch)` for test assertions
- `hal_ble_mock`: accumulates transmitted bytes in a ring buffer; provides `mock_ble_get_packet()` to retrieve transmitted packets

### 4.7 Integration Test (native platform, all mocks)

**First end-to-end test**: runs the full pipeline for 10 simulated seconds.

```
mock IMU → filter → tremor detector → PI controller → mock haptic → mock BLE
```

Assertions:
- Before t=0.3 s: `mock_haptic_get_drive(0) == 0` (state machine in IDLE, hold-off not elapsed)
- After t=0.5 s: `mock_haptic_get_drive(0) > 0` (ACTIVE, actuators driving)
- `mock_ble_get_packet()` every 50 ms, checksum valid
- During tremor-off period (t=10–15 s in mock): drive returns to 0 within 700 ms

---

## 5. Integration Milestones (Hardware)

Following the TDR 5.2 additive integration strategy:

| Step | What it verifies | Acceptance criterion |
|---|---|---|
| 0 | Software skeleton + native tests | All unit tests pass on host PC (`pio test -e native_test`) |
| 1 | STM32 → MPU-6050 I²C | Burst read returns non-zero gyro values; stable wrist angle in SWD watch |
| 2 | Bandpass filter on real IMU | 5 Hz wrist shake produces visible filtered output; slow tilt produces near-zero filtered output |
| 3 | RMS + state machine | LED toggles IDLE/ACTIVE correctly when shaking wrist |
| 4 | TCA9548A → DRV2605L → single motor | Motor vibrates on I²C command; intensity modulates with RTP byte |
| 5 | Close PI loop | Oscilloscope: drive byte proportional to shaking amplitude |
| 6 | HM-10 BLE | Live tremor graph visible in browser with MockTransport-equivalent data |
| 7 | Full wrist test | Measurable RMS reduction in before/after IMU data with glove worn |

---

## 6. Web App Build Order

1. **MockTransport** — generates synthetic telemetry packets in-browser at 20 Hz. No hardware needed. This is the primary demo fallback.
2. **Packet parser** — parses `TelemetryPacket` binary format from a `DataView`, validates checksum, handles header re-sync.
3. **Chart.js live graph** — rolling 10 s window showing: gyro magnitude, bandpass filtered signal, RMS envelope, tremor state (boolean overlay), drive intensity.
4. **CSV logger** — downloads a CSV with columns: `wall_clock_ms, device_timestamp_ms, gyro_magnitude, rms, tremor_active, drive_ch0, drive_ch1, drive_ch2`.
5. **FileTransport** — replays a CSV or binary log file at 20 Hz (see Q5 above).
6. **BleTransport** — Web Bluetooth connection to HM-10, GATT characteristic `0xFFE1`. Implement last; requires real hardware.

---

## 7. Risk Flags

### R1 — No SOFTWARE_DESIGN.md in repo [HIGH]
The brief references a `SOFTWARE_DESIGN.md` as the primary architecture document, but it does not exist in the repository. The `tremor_glove_software_system.pdf` is the closest equivalent. **Action**: Treat the brief itself + this PLAN.md as the authoritative software specification. Create `SOFTWARE_DESIGN.md` as the canonical reference after the scaffold is complete.

### R2 — STM32F401CCU6 CMSIS DSP struct names [MEDIUM]
The research notes reference `arm_biquad_cascade_df2T_f32` but the brief specifies `arm_biquad_casd_df2T_instance_f32` (typo in brief — likely `arm_biquad_casd_df2T_f32`). The actual CMSIS-DSP function is `arm_biquad_cascade_df2T_f32`. The instance struct is `arm_biquad_casd_df2T_instance_f32`. These must be confirmed against the actual CMSIS headers in the PlatformIO package before the filter is written.

### R3 — ERM vs LRA firmware difference [LOW]
ERM coin motors (placeholder) are driven via PWM through BC547 transistors. LRAs are driven via DRV2605L RTP mode over I²C. These are completely different electrical interfaces. The HAL abstraction (`hal_haptic_set_drive()`) hides this, but two separate HAL implementations are needed: `hal_haptic_erm_pwm.c` and `hal_haptic_drv2605l.c`. The compile-time flag `HAPTIC_ERM_PLACEHOLDER` selects which one is linked.

### R4 — TCA9548A channel deselect timing [MEDIUM]
After writing the RTP byte to a DRV2605L, the TCA9548A channel must be deselected (write 0x00) before selecting the next channel. If deselect is omitted, all channels on the bus respond simultaneously to the next write (all drivers share address 0x5A). This is an easy bug to make and hard to debug with a multimeter.

### R5 — Web Bluetooth API constraints [MEDIUM]
Web Bluetooth is only available in Chromium-based browsers (Chrome, Edge) and requires HTTPS or localhost. It is not available in Firefox, Safari, or any mobile browser except Chrome for Android. The web app must display a clear "not supported" message in unsupported browsers and fall back to MockTransport automatically.

### R6 — HM-10 BLE fragmentation [MEDIUM]
The HM-10 may split a 9-byte packet across two BLE notifications if the ATT MTU is smaller than 9 bytes (default ATT MTU is 23 bytes for BLE 4.0, so this is unlikely but possible). The parser must buffer incoming bytes and scan for the 0xAA sync byte rather than assuming one notification = one packet.

### R7 — 200 Hz loop jitter from I²C [MEDIUM]
If an I²C transaction is still in progress when SysTick fires the next loop iteration, the loop will either wait (violating the no-blocking rule) or proceed with stale data. The design uses DMA/interrupt-driven I²C. The implementation must set the new read in motion at the end of each cycle and use the data from the previous cycle's read — a classic double-buffer pattern. This must be explicitly verified during Step 1 integration.

---

## 8. Repository Structure

```
firmware/
  src/
    ring_buffer.c          # data structure, no HAL deps
    filters.c              # biquad bandpass, complementary filter, RMS
    tremor.c               # state machine, threshold config
    control.c              # PI controller with anti-windup
    telemetry.c            # packet serialise/validate
    main.c                 # init + 200 Hz loop
    hal/
      hal_imu.h            # interface: init, read → ImuData
      hal_haptic.h         # interface: init, set_drive(ch, intensity)
      hal_ble.h            # interface: init, transmit(buf, len)
      hal_imu_mpu6050.c    # concrete: STM32 I²C1 + MPU-6050
      hal_haptic_drv2605l.c # concrete: TCA9548A + DRV2605L via I²C2
      hal_haptic_erm_pwm.c  # concrete: ERM via PWM (placeholder)
      hal_ble_hm10.c        # concrete: UART1 DMA
      mock/
        hal_imu_mock.c     # synthetic tremor signal, native-only
        hal_haptic_mock.c  # records drive bytes, native-only
        hal_ble_mock.c     # accumulates TX bytes, native-only
  include/
    ring_buffer.h
    filters.h
    tremor.h
    control.h
    telemetry.h
    hal/
      hal_imu.h
      hal_haptic.h
      hal_ble.h
  test/
    test_ring_buffer.c
    test_filters.c
    test_tremor.c
    test_control.c
    test_telemetry.c
    test_integration.c
  platformio.ini            # stm32f401 + native_test environments

webapp/
  index.html
  app.js
  transport/
    mock_transport.js       # synthetic telemetry at 20 Hz
    file_transport.js       # replay from file at 20 Hz
    ble_transport.js        # Web Bluetooth → HM-10
  parser.js                 # TelemetryPacket binary parser
  chart.js                  # Chart.js live graph wrapper
  logger.js                 # CSV download

generate_coeffs.py          # offline biquad coefficient generation
README.md                   # setup guide
PLAN.md                     # this file
```

---

## 9. Telemetry Packet Format

13 bytes total, `__packed`, no padding. Based on the software design notes, adapted for IMU-only (EMG fields repurposed).

| Offset | Size | Field | Type | Notes |
|---|---|---|---|---|
| 0 | 1 | `header` | uint8_t | 0xAA sync byte |
| 1 | 2 | `timestamp_ms` | uint16_t | device uptime ms, wraps every ~65 s |
| 3 | 2 | `gyro_magnitude` | uint16_t | gyro vector magnitude × 100 (0.01 rad/s LSB) |
| 5 | 2 | `gyro_x` | int16_t | raw gyro X from IMU (raw register units) |
| 7 | 2 | `gyro_y` | int16_t | raw gyro Y from IMU |
| 9 | 1 | `tremor_active` | uint8_t | 0 = IDLE, 1 = ACTIVE |
| 10 | 1 | `drive_intensity` | uint8_t | CH0 (volar wrist) drive byte, 0–127 |
| 11 | 1 | `rms_x100` | uint8_t | RMS envelope × 100, saturates at 255 |
| 12 | 1 | `checksum` | uint8_t | XOR of bytes [1..11] |

Total = 1+2+2+2+2+1+1+1+1 = 13 bytes. Confirmed.

Checksum = `buf[1] ^ buf[2] ^ ... ^ buf[11]` (all bytes except header and checksum byte itself).

---

## 10. platformio.ini

```ini
[env:stm32f401]
platform = ststm32
board = blackpill_f401cc
framework = stm32cube
lib_deps =
    CMSIS-DSP
build_flags =
    -DHAPTIC_DRV2605L
    -DUSE_HAL_DRIVER

[env:native_test]
platform = native
test_framework = unity
build_flags =
    -DNATIVE_TEST
    -DHAPTIC_MOCK
    -std=c11
```

---

## 11. generate_coeffs.py Design

Run this **before** implementing filters.c to get the hardcoded coefficient arrays.

```python
from scipy.signal import butter
import numpy as np

FS = 200.0  # Hz (IMU sample rate)
F_LOW = 3.0
F_HIGH = 8.0

# 4th-order Butterworth bandpass
# Returns as second-order sections for cascaded biquad implementation
sos = butter(4, [F_LOW, F_HIGH], btype='bandpass', fs=FS, output='sos')

# Convert SOS to CMSIS-DSP Direct Form II Transposed coefficients
# CMSIS format per stage: [b0, b1, b2, a1, a2] with a0 normalised to 1
# NOTE: CMSIS uses -a1, -a2 (negated feedback) — check sign convention!
for i, stage in enumerate(sos):
    b0, b1, b2, a0, a1, a2 = stage
    print(f"  /* Stage {i} */ {b0/a0:.10f}f, {b1/a0:.10f}f, {b2/a0:.10f}f, "
          f"{-a1/a0:.10f}f, {-a2/a0:.10f}f,")
```

The output is pasted directly into `BANDPASS_COEFFS[]` in `filters.c`. The number of stages equals `len(sos)` (4 for a 4th-order bandpass — 2 poles each side = 4 biquad stages total).

---

## 12. Confirmed Design Decisions

| Item | Decision |
|---|---|
| Telemetry packet | 13-byte struct (see §9 above) |
| ERM PWM pin assignment | **Not finalised** — abstracted in HAL. Placeholder pins PA6/PA7/PB0 (TIM3 CH1/CH2/CH3). Not electrically committed. |
| HM-10 GATT | Service `0xFFE0`, characteristic `0xFFE1` (notify) |
| TCA9548A channel mapping | CH0 = volar wrist (100%), CH1 = volar forearm (70%), CH2 = dorsal wrist (50%) |
