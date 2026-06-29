# Tremor Suppression Glove — ELEC3117

Closed-loop vibrotactile tremor suppression glove for Parkinson's disease.
STM32F401CCU6 + MPU-6050 + 3× DRV2605L LRA actuators + HM-10 BLE.

---

## Firmware

### Prerequisites
- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- Python 3 + scipy/numpy (for coefficient generation only)

### 1. Generate filter coefficients (do this first)

```bash
python3 generate_coeffs.py
```

Paste the printed `BANDPASS_COEFFS` array into `firmware/src/filters.c`,
replacing the placeholder zeros.

### 2. Run unit tests (no hardware required)

```bash
pio test -e native_test
```

All five test suites (`ring_buffer`, `filters`, `tremor`, `control`, `telemetry`)
must pass before proceeding to hardware.

### 3. Build and flash (STM32F401 Black Pill)

Connect ST-Link. Then:

```bash
pio run -e stm32f401 --target upload
```

To use ERM coin motors instead of DRV2605L (early bring-up):

```bash
pio run -e stm32f401 --target upload -D HAPTIC_ERM_PLACEHOLDER
```

### 4. CubeMX peripheral init

`main.c` expects CubeMX-generated init functions (`SystemClock_Config`,
`MX_I2C1_Init`, `MX_I2C2_Init`, `MX_USART1_UART_Init`). Generate these
from the `.ioc` file in CubeIDE and place them in `firmware/src/`.

---

## Web App

No build step — runs directly in the browser as ES modules.

### Open the dashboard

```bash
python3 -m http.server 8080 --directory webapp/
```

Open `http://localhost:8080` in **Chrome or Edge** (required for Web Bluetooth).

### Transports

| Mode | How to use |
|---|---|
| **Mock** | Select "Mock (synthetic)" → Start. Works immediately, no hardware needed. |
| **File replay** | Select "File replay" → choose a `.bin` log → Start. |
| **BLE** | Select "BLE (real device)" → Start → approve the browser Bluetooth dialog. |

> Web Bluetooth requires Chrome/Edge. Does **not** work in Firefox or Safari.

### Download data

Click **Download CSV** at any time to save the session log with wall-clock timestamps.

---

## Repository structure

```
firmware/
  include/          # Public headers (ring_buffer, filters, tremor, control, telemetry)
  include/hal/      # HAL interface headers (platform-agnostic)
  src/              # Algorithm implementations (compile on any platform)
  src/hal/          # Concrete HAL drivers (STM32 target only)
  src/hal/mock/     # Mock HAL for native unit testing
  test/             # Unity unit tests (pio test -e native_test)
webapp/
  index.html        # Dashboard UI
  app.js            # Entry point — wires transport → chart → logger
  parser.js         # Binary packet parser with header re-sync
  chart_view.js     # Chart.js rolling 10-second graph
  logger.js         # CSV download
  transport/        # MockTransport, FileTransport, BleTransport
generate_coeffs.py  # Biquad coefficient generator (run before building)
PLAN.md             # Full implementation plan and design decisions
platformio.ini      # Build environments: stm32f401 and native_test
```

---

## Integration milestones

| Step | Acceptance criterion |
|---|---|
| 0 — Native tests | `pio test -e native_test` all pass |
| 1 — IMU | Stable wrist angle in SWD live watch |
| 2 — Filter | 5 Hz shake → clear filtered output; slow tilt → near zero |
| 3 — State machine | LED toggles IDLE/ACTIVE on wrist shake |
| 4 — Haptic | Motor vibrates; intensity modulates with RTP byte |
| 5 — PI loop | Drive byte proportional to shake amplitude (oscilloscope) |
| 6 — BLE | Live tremor graph visible in browser |
| 7 — Full test | Measurable RMS reduction before/after with glove worn |
