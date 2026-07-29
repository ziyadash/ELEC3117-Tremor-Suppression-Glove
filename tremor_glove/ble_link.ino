#include "ble_link.h"
#include <string.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* Same UUIDs as esp32-mpu6050-prototype - the web app connects to either
   firmware unmodified (aside from expecting the longer packet here). */
#define SERVICE_UUID        "a1b2c3d4-0001-4a5b-8c6d-1234567890ab"
#define CHARACTERISTIC_UUID "a1b2c3d4-0002-4a5b-8c6d-1234567890ab"

static BLEServer *s_server = nullptr;
static BLECharacteristic *s_char = nullptr;
static bool s_connected = false;

class LinkServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    (void)server;
    s_connected = true;
  }
  void onDisconnect(BLEServer *server) override {
    s_connected = false;
    server->getAdvertising()->start();
  }
};

void ble_link_init(const char *device_name) {
  BLEDevice::init(device_name);
  /* Matches esp32-mpu6050-prototype's negotiated MTU; our 55-byte packet
     needs more than the default 20-byte notify payload. */
  BLEDevice::setMTU(247);

  s_server = BLEDevice::createServer();
  s_server->setCallbacks(new LinkServerCallbacks());

  BLEService *service = s_server->createService(SERVICE_UUID);
  s_char = service->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  s_char->addDescriptor(new BLE2902());
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

bool ble_link_connected() { return s_connected; }

void ble_link_send(float qw, float qx, float qy, float qz,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    bool tremor_active, uint8_t lead_channel,
                    float rms, float threshold, float freq_hz,
                    uint8_t drive_intensity) {
  if (!s_connected) return;

  uint8_t buf[BLE_PACKET_SIZE];
  float floats[10] = { qw, qx, qy, qz, ax, ay, az, gx, gy, gz };
  memcpy(buf, floats, sizeof(floats));

  buf[40] = tremor_active ? 1 : 0;
  buf[41] = lead_channel;
  memcpy(buf + 42, &rms, sizeof(float));
  memcpy(buf + 46, &threshold, sizeof(float));
  memcpy(buf + 50, &freq_hz, sizeof(float));
  buf[54] = drive_intensity;

  s_char->setValue(buf, BLE_PACKET_SIZE);
  s_char->notify();
}
