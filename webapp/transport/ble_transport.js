/**
 * BleTransport — Web Bluetooth connection to HM-10 module.
 *
 * Service UUID:         0000ffe0-0000-1000-8000-00805f9b34fb  (0xFFE0)
 * Characteristic UUID: 0000ffe1-0000-1000-8000-00805f9b34fb  (0xFFE1, notify)
 *
 * Handles BLE notification fragmentation: accumulates bytes in a buffer and
 * scans for the 0xAA header, validating each complete packet before delivery.
 *
 * Requires Chrome/Edge and HTTPS or localhost.
 */
import { PACKET_SIZE, HEADER_BYTE, extractPackets } from '../parser.js';

const SERVICE_UUID = '0000ffe0-0000-1000-8000-00805f9b34fb';
const CHAR_UUID    = '0000ffe1-0000-1000-8000-00805f9b34fb';

export class BleTransport {
  constructor({ onPacket, onStatus } = {}) {
    this._onPacket   = onPacket  ?? (() => {});
    this._onStatus   = onStatus  ?? (() => {});
    this._device     = null;
    this._char       = null;
    this._rxBuffer   = new Uint8Array(0);
  }

  static isSupported() {
    return 'bluetooth' in navigator;
  }

  async connect() {
    if (!BleTransport.isSupported()) {
      this._onStatus('Web Bluetooth not supported in this browser');
      return false;
    }

    try {
      this._onStatus('Scanning…');
      this._device = await navigator.bluetooth.requestDevice({
        filters: [{ namePrefix: 'HM' }],
        optionalServices: [SERVICE_UUID],
      });

      this._device.addEventListener('gattserverdisconnected', () => {
        this._onStatus('BLE disconnected');
      });

      const server  = await this._device.gatt.connect();
      const service = await server.getPrimaryService(SERVICE_UUID);
      this._char    = await service.getCharacteristic(CHAR_UUID);

      this._char.addEventListener('characteristicvaluechanged',
                                  (e) => this._onNotify(e));
      await this._char.startNotifications();

      this._onStatus(`Connected: ${this._device.name}`);
      return true;
    } catch (err) {
      this._onStatus(`BLE error: ${err.message}`);
      return false;
    }
  }

  async disconnect() {
    if (this._char) await this._char.stopNotifications().catch(() => {});
    if (this._device?.gatt.connected) this._device.gatt.disconnect();
    this._onStatus('Disconnected');
  }

  _onNotify(event) {
    const incoming = new Uint8Array(event.target.value.buffer);

    /* Append to accumulation buffer */
    const merged = new Uint8Array(this._rxBuffer.length + incoming.length);
    merged.set(this._rxBuffer);
    merged.set(incoming, this._rxBuffer.length);

    const view = new DataView(merged.buffer);
    const { packets, consumed } = extractPackets(view);

    packets.forEach(p => this._onPacket(p));

    /* Keep unconsumed bytes for next notification */
    this._rxBuffer = merged.slice(consumed);
  }
}
