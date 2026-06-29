/**
 * MockTransport — generates synthetic telemetry at 20 Hz.
 *
 * Signal: tremor_amp * sin(2π * tremor_hz * t)
 *       + voluntary_amp * sin(2π * 0.8 * t)
 *       + gaussian noise
 *
 * Calls onPacket(parsedPacket) every 50 ms.
 * Tremor is on for 10 s, off for 5 s, cycling continuously.
 */
export class MockTransport {
  constructor({ onPacket, onStatus } = {}) {
    this._onPacket  = onPacket  ?? (() => {});
    this._onStatus  = onStatus  ?? (() => {});
    this._timer     = null;
    this._t         = 0;         // simulated time in seconds
    this._tickMs    = 0;
    this._tremorHz  = 5.0;
    this._tremorAmp = 0.25;      // rad/s
    this._volAmp    = 0.05;
    this._noiseAmp  = 0.02;
    this._cycleMs   = 15000;     // 15 s full cycle
    this._onMs      = 10000;     // tremor on for first 10 s of cycle
  }

  start() {
    this._t      = 0;
    this._tickMs = 0;
    this._onStatus('Mock running');
    this._timer = setInterval(() => this._tick(), 50);
  }

  stop() {
    clearInterval(this._timer);
    this._timer = null;
    this._onStatus('Stopped');
  }

  _tick() {
    const DT = 0.05;
    this._t      += DT;
    this._tickMs += 50;

    const cyclePos = this._tickMs % this._cycleMs;
    const tremorOn = cyclePos < this._onMs;

    const tremor    = tremorOn
      ? this._tremorAmp * Math.sin(2 * Math.PI * this._tremorHz * this._t)
      : 0;
    const voluntary = this._volAmp * Math.sin(2 * Math.PI * 0.8 * this._t);
    const noise     = (Math.random() - 0.5) * 2 * this._noiseAmp;

    const gyro_y    = tremor + voluntary + noise;
    const gyroMag   = Math.abs(gyro_y);
    // Simple mock RMS: exponential moving average of gyroMag² → sqrt
    this._rmsState  = (this._rmsState ?? 0) * 0.95 + gyroMag * gyroMag * 0.05;
    const rms       = Math.sqrt(this._rmsState);

    const pkt = {
      timestamp_ms:    this._tickMs & 0xFFFF,
      gyro_magnitude:  gyroMag,
      gyro_x:          Math.round(noise * 65.5 / 0.017453293),
      gyro_y:          Math.round(gyro_y * 65.5 / 0.017453293),
      tremor_active:   tremorOn && rms > 0.08,
      drive_intensity: tremorOn ? Math.min(127, Math.round(rms * 200)) : 0,
      rms,
    };
    this._onPacket(pkt);
  }
}
