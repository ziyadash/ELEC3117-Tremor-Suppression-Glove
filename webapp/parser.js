/**
 * TelemetryPacket binary parser.
 *
 * Packet layout (13 bytes):
 *   [0]     header          0xAA
 *   [1-2]   timestamp_ms    uint16 LE
 *   [3-4]   gyro_magnitude  uint16 LE  (× 0.01 = rad/s)
 *   [5-6]   gyro_x          int16  LE  (raw register units)
 *   [7-8]   gyro_y          int16  LE
 *   [9]     tremor_active   uint8
 *   [10]    drive_intensity uint8  (CH0, 0–127)
 *   [11]    rms_x100        uint8  (× 0.01 = rad/s)
 *   [12]    checksum        uint8  XOR of bytes [1..11]
 */

export const PACKET_SIZE = 13;
export const HEADER_BYTE = 0xAA;

export function validatePacket(view, offset = 0) {
  if (view.byteLength - offset < PACKET_SIZE) return false;
  if (view.getUint8(offset) !== HEADER_BYTE) return false;

  let cs = 0;
  for (let i = 1; i <= 11; i++) cs ^= view.getUint8(offset + i);
  return cs === view.getUint8(offset + 12);
}

export function parsePacket(view, offset = 0) {
  return {
    timestamp_ms:    view.getUint16(offset + 1,  true),
    gyro_magnitude:  view.getUint16(offset + 3,  true) * 0.01,
    gyro_x:          view.getInt16 (offset + 5,  true),
    gyro_y:          view.getInt16 (offset + 7,  true),
    tremor_active:   view.getUint8 (offset + 9)  === 1,
    drive_intensity: view.getUint8 (offset + 10),
    rms:             view.getUint8 (offset + 11) * 0.01,
  };
}

/**
 * Scan a DataView for valid packets, handling fragmentation.
 * Returns an array of parsed packet objects and the number of bytes consumed.
 */
export function extractPackets(view) {
  const packets = [];
  let i = 0;
  while (i <= view.byteLength - PACKET_SIZE) {
    if (view.getUint8(i) === HEADER_BYTE && validatePacket(view, i)) {
      packets.push(parsePacket(view, i));
      i += PACKET_SIZE;
    } else {
      i++; /* re-sync: scan byte-by-byte for next header */
    }
  }
  return { packets, consumed: i };
}
