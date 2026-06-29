/**
 * CSV logger.
 * Accumulates packets and allows downloading as CSV with wall-clock timestamps.
 *
 * Columns: wall_clock_ms, device_timestamp_ms, gyro_magnitude,
 *          rms, tremor_active, drive_ch0
 */
export class Logger {
  constructor() {
    this._rows = [];
  }

  record(pkt) {
    this._rows.push({
      wall_clock_ms:       Date.now(),
      device_timestamp_ms: pkt.timestamp_ms,
      gyro_magnitude:      pkt.gyro_magnitude.toFixed(4),
      rms:                 pkt.rms.toFixed(4),
      tremor_active:       pkt.tremor_active ? 1 : 0,
      drive_ch0:           pkt.drive_intensity,
    });
  }

  clear() { this._rows = []; }

  download(filename = 'tremor_log.csv') {
    if (this._rows.length === 0) return;

    const header = Object.keys(this._rows[0]).join(',');
    const lines  = this._rows.map(r => Object.values(r).join(','));
    const csv    = [header, ...lines].join('\n');

    const blob = new Blob([csv], { type: 'text/csv' });
    const url  = URL.createObjectURL(blob);
    const a    = document.createElement('a');
    a.href     = url;
    a.download = filename;
    a.click();
    URL.revokeObjectURL(url);
  }

  get count() { return this._rows.length; }
}
