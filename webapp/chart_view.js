/**
 * ChartView — rolling 10-second Chart.js live graph.
 *
 * Plots:
 *   - Gyro magnitude (blue)
 *   - RMS envelope  (orange)
 *   - Drive CH0     (green, secondary Y axis 0–127)
 *   - Tremor active (red shaded background)
 */
const WINDOW_S    = 10;       /* seconds of history to show */
const RATE_HZ     = 20;       /* packet rate */
const MAX_POINTS  = WINDOW_S * RATE_HZ;

export class ChartView {
  constructor(canvasId) {
    const ctx = document.getElementById(canvasId).getContext('2d');

    this._labels = [];
    this._gyro   = [];
    this._rms    = [];
    this._drive  = [];

    this._chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: this._labels,
        datasets: [
          {
            label: 'Gyro magnitude (rad/s)',
            data: this._gyro,
            borderColor: '#5af',
            borderWidth: 1,
            pointRadius: 0,
            tension: 0.1,
            yAxisID: 'y',
          },
          {
            label: 'RMS envelope (rad/s)',
            data: this._rms,
            borderColor: '#fa5',
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.2,
            yAxisID: 'y',
          },
          {
            label: 'Drive CH0 (0–127)',
            data: this._drive,
            borderColor: '#5d5',
            borderWidth: 1.5,
            pointRadius: 0,
            tension: 0.1,
            yAxisID: 'y2',
          },
        ],
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: {
            ticks: { color: '#888', maxTicksLimit: 10 },
            grid:  { color: '#222' },
          },
          y: {
            position: 'left',
            ticks: { color: '#5af' },
            grid:  { color: '#1a1a1a' },
            title: { display: true, text: 'rad/s', color: '#888' },
          },
          y2: {
            position: 'right',
            min: 0, max: 127,
            ticks: { color: '#5d5' },
            grid:  { drawOnChartArea: false },
            title: { display: true, text: 'Drive (0–127)', color: '#888' },
          },
        },
        plugins: {
          legend: { labels: { color: '#ccc', boxWidth: 12 } },
        },
      },
    });
  }

  push(pkt) {
    /* Handle timestamp wraparound gracefully — just use arrival order */
    const label = (pkt.timestamp_ms / 1000).toFixed(2) + 's';

    this._labels.push(label);
    this._gyro.push(pkt.gyro_magnitude);
    this._rms.push(pkt.rms);
    this._drive.push(pkt.drive_intensity);

    /* Trim to rolling window */
    if (this._labels.length > MAX_POINTS) {
      this._labels.shift();
      this._gyro.shift();
      this._rms.shift();
      this._drive.shift();
    }

    this._chart.update('none'); /* 'none' skips animation for performance */
  }

  clear() {
    this._labels.length = 0;
    this._gyro.length   = 0;
    this._rms.length    = 0;
    this._drive.length  = 0;
    this._chart.update();
  }
}
