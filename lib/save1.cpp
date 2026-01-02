#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ================= PIN CONFIG =================
#define RELAY_PIN 4          // Relay (lampu / beban)
#define FAN_PIN   16         // Fan 5V (via relay / MOSFET)

#define RELAY_ACTIVE_LOW true
#define FAN_ACTIVE_HIGH  true

// ================= BLE UUID ===================
#define SERVICE_UUID        "0001046d-9d7a-4cc4-8cd9-e385bc31104d"
#define CHARACTERISTIC_UUID "babc1b75-f950-4e74-8248-5e984a9efb09"

// ================= BLE STATE ==================
static BLEAdvertisedDevice* myDevice = nullptr;
static BLERemoteCharacteristic* remoteChar = nullptr;
static BLEClient* client = nullptr;
static bool connected = false;
static bool doConnect = false;

// ================= UTIL ======================
void relayWrite(bool on) {
  if (RELAY_ACTIVE_LOW)
    digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  else
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

void fanWrite(bool on) {
  if (FAN_ACTIVE_HIGH)
    digitalWrite(FAN_PIN, on ? HIGH : LOW);
  else
    digitalWrite(FAN_PIN, on ? LOW : HIGH);
}

// ================= NOTIFY CALLBACK =================
static void notifyCallback(
  BLERemoteCharacteristic*,
  uint8_t* data,
  size_t length,
  bool) {

  if (length < 1) return;

  uint8_t code = data[0];

  Serial.print("BLE CODE = ");
  Serial.println(code);

  switch (code) {
    case 0:
      relayWrite(false);
      fanWrite(false);
      Serial.println("ALL OFF");
      break;

    case 1:
      relayWrite(true);
      fanWrite(false);
      Serial.println("RELAY ON");
      break;

    case 2:
      relayWrite(false);
      fanWrite(true);
      Serial.println("FAN ON");
      break;

    default:
      Serial.println("UNKNOWN CODE");
      break;
  }
}

// ================= SCAN CALLBACK =================
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {

      Serial.println("Found BLE Sender");
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      BLEDevice::getScan()->stop();
    }
  }
};

// ================= CONNECT =================
bool connectToServer() {
  Serial.println("Connecting to BLE server...");
  client = BLEDevice::createClient();

  if (!client->connect(myDevice)) {
    Serial.println("Connection failed");
    return false;
  }

  BLERemoteService* service = client->getService(BLEUUID(SERVICE_UUID));
  if (!service) return false;

  remoteChar = service->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (!remoteChar) return false;

  if (remoteChar->canNotify()) {
    remoteChar->registerForNotify(notifyCallback);
  } else {
    return false;
  }

  connected = true;
  Serial.println("BLE CONNECTED ✅");
  return true;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  relayWrite(false);
  fanWrite(false);

  BLEDevice::init("ESP32-Receiver");

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setActiveScan(true);
  scan->start(0, false);

  Serial.println("Scanning BLE...");
}

// ================= LOOP =================
void loop() {
  if (doConnect && !connected) {
    doConnect = false;
    if (!connectToServer()) {
      Serial.println("Retry scan...");
      BLEDevice::getScan()->start(0, false);
    }
  }

  if (connected && client && !client->isConnected()) {
    Serial.println("BLE Disconnected, rescan...");
    connected = false;
    BLEDevice::getScan()->start(0, false);
  }

  delay(200);
}
