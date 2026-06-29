/**
 * FileTransport — replays a binary telemetry log file at exactly 20 Hz.
 *
 * Accepts a .bin file containing raw 13-byte telemetry packets concatenated.
 * Delivers one packet every 50 ms regardless of how many packets the file
 * contains (throttle to 20 Hz; if file has fewer, pauses between packets).
 *
 * Calls onPacket(parsedPacket) and onStatus(msg).
 */
import { PACKET_SIZE, validatePacket, parsePacket } from '../parser.js';

export class FileTransport {
  constructor({ onPacket, onStatus } = {}) {
    this._onPacket = onPacket  ?? (() => {});
    this._onStatus = onStatus  ?? (() => {});
    this._timer    = null;
    this._packets  = [];
    this._index    = 0;
  }

  async loadFile(file) {
    const buf    = await file.arrayBuffer();
    const view   = new DataView(buf);
    this._packets = [];

    let i = 0;
    while (i <= view.byteLength - PACKET_SIZE) {
      if (validatePacket(view, i)) {
        this._packets.push(parsePacket(view, i));
        i += PACKET_SIZE;
      } else {
        i++; /* skip and re-sync */
      }
    }

    this._onStatus(`Loaded ${this._packets.length} packets from ${file.name}`);
    return this._packets.length;
  }

  start() {
    if (this._packets.length === 0) {
      this._onStatus('No packets loaded');
      return;
    }
    this._index = 0;
    this._onStatus(`Replaying ${this._packets.length} packets at 20 Hz`);
    this._timer = setInterval(() => this._tick(), 50);
  }

  stop() {
    clearInterval(this._timer);
    this._timer = null;
    this._onStatus('Stopped');
  }

  _tick() {
    if (this._index >= this._packets.length) {
      this.stop();
      this._onStatus('Replay complete');
      return;
    }
    this._onPacket(this._packets[this._index++]);
  }
}
