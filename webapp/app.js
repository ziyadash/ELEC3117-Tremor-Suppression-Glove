/**
 * app.js — main entry point for the tremor glove dashboard.
 *
 * Wires together: transport selection, packet routing, chart, logger.
 */
import { MockTransport }  from './transport/mock_transport.js';
import { FileTransport }  from './transport/file_transport.js';
import { BleTransport }   from './transport/ble_transport.js';
import { ChartView }      from './chart_view.js';
import { Logger }         from './logger.js';

const chart  = new ChartView('chart');
const logger = new Logger();

let transport     = null;
let packetCount   = 0;

/* ---- DOM refs ---- */
const sel        = document.getElementById('transport-select');
const fileInput  = document.getElementById('file-input');
const btnStart   = document.getElementById('btn-start');
const btnStop    = document.getElementById('btn-stop');
const btnDl      = document.getElementById('btn-download');
const statusEl   = document.getElementById('status');
const sGyro      = document.getElementById('s-gyro');
const sRms       = document.getElementById('s-rms');
const sState     = document.getElementById('s-state');
const sDrive     = document.getElementById('s-drive');
const sCount     = document.getElementById('s-count');

/* ---- Packet handler ---- */
function onPacket(pkt) {
  chart.push(pkt);
  logger.record(pkt);
  packetCount++;

  sGyro.textContent  = pkt.gyro_magnitude.toFixed(3);
  sRms.textContent   = pkt.rms.toFixed(3);
  sState.textContent = pkt.tremor_active ? 'ACTIVE' : 'IDLE';
  sDrive.textContent = pkt.drive_intensity;
  sCount.textContent = packetCount;
}

function setStatus(msg, cls = '') {
  statusEl.textContent  = msg;
  statusEl.className    = cls;
}

/* ---- Transport selection ---- */
sel.addEventListener('change', () => {
  fileInput.style.display = sel.value === 'file' ? 'inline' : 'none';
});

/* ---- Start ---- */
btnStart.addEventListener('click', async () => {
  if (transport) { transport.stop?.(); transport = null; }

  packetCount = 0;
  logger.clear();
  chart.clear();

  const opts = { onPacket, onStatus: (msg) => setStatus(msg, 'ok') };

  switch (sel.value) {
    case 'mock':
      transport = new MockTransport(opts);
      transport.start();
      break;

    case 'file': {
      const file = fileInput.files[0];
      if (!file) { setStatus('Select a file first', 'warn'); return; }
      const ft = new FileTransport(opts);
      const n  = await ft.loadFile(file);
      if (n === 0) { setStatus('No valid packets in file', 'warn'); return; }
      transport = ft;
      transport.start();
      break;
    }

    case 'ble':
      if (!BleTransport.isSupported()) {
        setStatus('Web Bluetooth not supported — use Chrome/Edge', 'warn');
        return;
      }
      transport = new BleTransport(opts);
      await transport.connect();
      break;
  }

  btnStart.disabled = true;
  btnStop.disabled  = false;
});

/* ---- Stop ---- */
btnStop.addEventListener('click', () => {
  transport?.stop?.();
  transport?.disconnect?.();
  transport = null;
  btnStart.disabled = false;
  btnStop.disabled  = true;
  setStatus('Stopped');
});

/* ---- Download ---- */
btnDl.addEventListener('click', () => logger.download());

/* ---- Auto-start with mock for immediate demo ---- */
setStatus('Ready — click Start');
