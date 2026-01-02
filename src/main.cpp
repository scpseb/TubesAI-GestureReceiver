#include <Arduino.h>

// ================= BLYNK CONFIG =================
#define BLYNK_TEMPLATE_ID   "TMPL6BVZEF-iH"
#define BLYNK_TEMPLATE_NAME "TubesIoT"
#define BLYNK_AUTH_TOKEN    "NdGClqTkK2-MF8ak6khBpEWFvMSS2Pmq"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// BLE Libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ================= WIFI CONFIG =================
const char* ssid     = "NAMA_WIFI";      // Ganti dengan SSID WiFi
const char* password = "PASSWORD_WIFI";  // Ganti dengan password WiFi

#define RELAY_PIN       2    
#define FAN_PIN         16 

// Pin behavior

#define RELAY_ACTIVE_LOW  false    // Set false jika relay Anda active high
#define FAN_ACTIVE_HIGH   true    // Fan ON saat pin HIGH

// ================= BLE UUID ===================
#define SERVICE_UUID        "0001046d-9d7a-4cc4-8cd9-e385bc31104d"
#define CHARACTERISTIC_UUID "babc1b75-f950-4e74-8248-5e984a9efb09"

// ================= BLYNK VIRTUAL PINS =========
#define VPIN_RELAY_STATUS   V0    // Status relay (output)
#define VPIN_FAN_STATUS     V1    // Status fan (output)
#define VPIN_RELAY_CONTROL  V2    // Kontrol relay dari app (input)
#define VPIN_FAN_CONTROL    V3    // Kontrol fan dari app (input)

// ================= TIMING CONFIG ==============
#define BLE_SCAN_INTERVAL   5000   // Interval scan BLE (ms)
#define BLYNK_SYNC_INTERVAL 1000   // Interval sync ke Blynk (ms)
#define RECONNECT_DELAY     3000   // Delay sebelum reconnect (ms)

// ================= GLOBAL STATE ===============
// Device states
static bool relayState = false;
static bool fanState = false;
static uint8_t lastGestureCode = 255;  // 255 = belum ada

// BLE states
static BLEAdvertisedDevice* myDevice = nullptr;
static BLERemoteCharacteristic* remoteChar = nullptr;
static BLEClient* client = nullptr;
static bool bleConnected = false;
static bool doConnect = false;
static bool scanning = false;

// Timing
static unsigned long lastBlynkSync = 0;
static unsigned long lastBLECheck = 0;

// ================= FUNCTION DECLARATIONS ======
void relayWrite(bool on);
void fanWrite(bool on);
void updateBlynkStatus();
void handleGestureCode(uint8_t code);
bool connectToServer();
void startBLEScan();

// ================= RELAY & FAN CONTROL ========
void relayWrite(bool on) {
    relayState = on;
    
    // ACTIVE LOW: Relay ON saat pin LOW, Relay OFF saat pin HIGH
    if (RELAY_ACTIVE_LOW) {
        digitalWrite(RELAY_PIN, on ? LOW : HIGH);
    } else {
        digitalWrite(RELAY_PIN, on ? HIGH : LOW);
    }
    
    Serial.printf("[Relay] %s (pin=%d)\n", on ? "ON" : "OFF", digitalRead(RELAY_PIN));
}

void fanWrite(bool on) {
    fanState = on;
    
    if (FAN_ACTIVE_HIGH) {
        digitalWrite(FAN_PIN, on ? HIGH : LOW);
    } else {
        digitalWrite(FAN_PIN, on ? LOW : HIGH);
    }
    
    Serial.printf("[Fan] %s (pin=%d)\n", on ? "ON" : "OFF", digitalRead(FAN_PIN));
}

// ================= GESTURE HANDLER ============
void handleGestureCode(uint8_t code) {
    Serial.printf("\n[Gesture] Received code: %d\n", code);
    lastGestureCode = code;
    
    switch (code) {
        case 0:  // 0 Jari - Matikan semua
            relayWrite(false);
            fanWrite(false);
            Serial.println("[Action] ALL OFF");
            break;
            
        case 1:  // 1 Jari - Nyalakan Relay/Lampu
            relayWrite(true);
            fanWrite(false);
            Serial.println("[Action] RELAY ON, FAN OFF");
            break;
            
        case 2:  // 2 Jari - Nyalakan Fan
            relayWrite(false);
            fanWrite(true);
            Serial.println("[Action] RELAY OFF, FAN ON");
            break;
            
        default:
            Serial.printf("[Action] Unknown code: %d\n", code);
            break;
    }
    
    // Update Blynk immediately
    Blynk.virtualWrite(VPIN_RELAY_STATUS, relayState ? 1 : 0);
    Blynk.virtualWrite(VPIN_FAN_STATUS, fanState ? 1 : 0);
}

// ================= BLYNK CALLBACKS ============

// Kontrol Relay dari Blynk App
BLYNK_WRITE(VPIN_RELAY_CONTROL) {
    int value = param.asInt();
    Serial.printf("[Blynk] Relay control: %d\n", value);
    relayWrite(value == 1);
    Blynk.virtualWrite(VPIN_RELAY_STATUS, relayState ? 1 : 0);
}

// Kontrol Fan dari Blynk App
BLYNK_WRITE(VPIN_FAN_CONTROL) {
    int value = param.asInt();
    Serial.printf("[Blynk] Fan control: %d\n", value);
    fanWrite(value == 1);
    Blynk.virtualWrite(VPIN_FAN_STATUS, fanState ? 1 : 0);
}

// Sinkronisasi saat app terhubung
BLYNK_CONNECTED() {
    Serial.println("[Blynk] Connected to server");
    
    // Sync semua status ke app
    Blynk.virtualWrite(VPIN_RELAY_STATUS, relayState ? 1 : 0);
    Blynk.virtualWrite(VPIN_FAN_STATUS, fanState ? 1 : 0);
}

// Update status ke Blynk secara periodik
void updateBlynkStatus() {
    if (millis() - lastBlynkSync >= BLYNK_SYNC_INTERVAL) {
        lastBlynkSync = millis();
        
        Blynk.virtualWrite(VPIN_RELAY_STATUS, relayState ? 1 : 0);
        Blynk.virtualWrite(VPIN_FAN_STATUS, fanState ? 1 : 0);
    }
}

// ================= BLE NOTIFY CALLBACK ========
static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* data,
    size_t length,
    bool isNotify) 
{
    if (length < 1) return;
    
    uint8_t code = data[0];
    handleGestureCode(code);
}

// ================= BLE SCAN CALLBACK ==========
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        // Hanya log device yang punya nama atau service UUID
        if (advertisedDevice.haveName() || advertisedDevice.haveServiceUUID()) {
            Serial.printf("[BLE Scan] Found: %s\n", advertisedDevice.toString().c_str());
        }
        
        // PENTING: Cek HANYA device dengan Service UUID yang TEPAT
        // dan nama yang sesuai (Gesture-XIAO)
        bool hasCorrectUUID = advertisedDevice.haveServiceUUID() &&
                              advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID));
        
        bool hasCorrectName = advertisedDevice.haveName() &&
                              (strcmp(advertisedDevice.getName().c_str(), "Gesture-XIAO") == 0);
        
        // Harus punya UUID yang benar, atau nama yang benar
        if (hasCorrectUUID || hasCorrectName) 
        {
            Serial.println("[BLE Scan] ✅ Found Gesture-XIAO Sender!");
            Serial.printf("[BLE Scan] Address: %s\n", advertisedDevice.getAddress().toString().c_str());
            
            // Simpan device
            if (myDevice != nullptr) {
                delete myDevice;
            }
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            scanning = false;
            
            // Stop scanning
            BLEDevice::getScan()->stop();
        }
    }
};

// ================= BLE CLIENT CALLBACK ========
class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        Serial.println("[BLE] Client connected callback");
        // JANGAN ubah state relay/fan disini!
    }

    void onDisconnect(BLEClient* pclient) override {
        bleConnected = false;
        Serial.println("[BLE] ❌ Disconnected from server");
                
        // LED status OFF (hanya LED indikator)
        //digitalWrite(LED_STATUS_PIN, LOW);
    }
};

// ================= BLE CONNECT ================
bool connectToServer() {
    if (myDevice == nullptr) {
        Serial.println("[BLE] No device to connect");
        return false;
    }
    
    Serial.printf("[BLE] Connecting to: %s\n", myDevice->getAddress().toString().c_str());
    
    // Buat client baru jika belum ada
    if (client == nullptr) {
        client = BLEDevice::createClient();
        client->setClientCallbacks(new MyClientCallback());
    }
    
    // Connect ke server
    if (!client->connect(myDevice)) {
        Serial.println("[BLE] ❌ Connection failed");
        return false;
    }
    Serial.println("[BLE] Connected to server");
    
    // Dapatkan service
    BLERemoteService* remoteService = client->getService(BLEUUID(SERVICE_UUID));
    if (remoteService == nullptr) {
        Serial.println("[BLE] ❌ Service not found");
        client->disconnect();
        return false;
    }
    Serial.println("[BLE] Service found");
    
    // Dapatkan characteristic
    remoteChar = remoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
    if (remoteChar == nullptr) {
        Serial.println("[BLE] ❌ Characteristic not found");
        client->disconnect();
        return false;
    }
    Serial.println("[BLE] Characteristic found");
    
    // Register untuk notify
    if (remoteChar->canNotify()) {
        remoteChar->registerForNotify(notifyCallback);
        Serial.println("[BLE] ✅ Registered for notifications");
    } else {
        Serial.println("[BLE] ❌ Cannot register for notify");
        client->disconnect();
        return false;
    }
    
    bleConnected = true;

    
    Serial.println("[BLE] ✅ FULLY CONNECTED");
    return true;
}

// ================= BLE SCAN START =============
void startBLEScan() {
    if (scanning) return;
    
    Serial.println("[BLE] Starting scan for Gesture-XIAO...");
    scanning = true;
    doConnect = false;  // Reset flag
    
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);  // true = delete old callbacks
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    
    // Scan async, akan memanggil callback saat selesai
    pBLEScan->start(10, [](BLEScanResults results) {
        Serial.printf("[BLE Scan] Complete. Found %d devices\n", results.getCount());
        scanning = false;
        
        // Jika tidak menemukan target, doConnect tetap false
        if (!doConnect) {
            Serial.println("[BLE Scan] Gesture-XIAO not found, will retry...");
        }
    }, false);
}

// ================= SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=============================================");
    Serial.println("  ESP32 BLE Receiver + Blynk Integration");
    Serial.println("=============================================\n");
    
    digitalWrite(RELAY_PIN, LOW);  // Pre-set HIGH (relay OFF)
    digitalWrite(FAN_PIN, LOW);     // Pre-set LOW (fan OFF)
    
    // Baru set pinMode setelah state sudah benar
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    
    // Force pin state again after pinMode
    digitalWrite(RELAY_PIN, HIGH);  // Pastikan HIGH (relay OFF)
    digitalWrite(FAN_PIN, LOW);     // Pastikan LOW (fan OFF)
    
    // Konfirmasi state awal
    Serial.printf("[Setup] RELAY_PIN GPIO%d = %d (HIGH=OFF for active-low relay)\n", RELAY_PIN, digitalRead(RELAY_PIN));
    Serial.printf("[Setup] FAN_PIN GPIO%d = %d (LOW=OFF)\n", FAN_PIN, digitalRead(FAN_PIN));
    
    // ============ TEST RELAY ============
    Serial.println("\n[TEST] Testing relay control...");
    Serial.println("[TEST] Relay should be OFF now. Check your relay.");
    delay(2000);
    
    Serial.println("[TEST] Turning relay ON (pin LOW)...");
    digitalWrite(RELAY_PIN, LOW);
    Serial.printf("[TEST] RELAY_PIN = %d\n", digitalRead(RELAY_PIN));
    delay(2000);
    
    Serial.println("[TEST] Turning relay OFF (pin HIGH)...");
    digitalWrite(RELAY_PIN, HIGH);
    Serial.printf("[TEST] RELAY_PIN = %d\n", digitalRead(RELAY_PIN));
    delay(1000);
    
    Serial.println("[TEST] Relay test complete.\n");
    // ============ END TEST ============
    
    // Set initial state melalui fungsi (untuk update variable juga)
    relayWrite(false);
    fanWrite(false);
    
    // Connect WiFi
    Serial.printf("[WiFi] Connecting to %s", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    int wifiRetry = 0;
    while (WiFi.status() != WL_CONNECTED && wifiRetry < 30) {
        delay(500);
        Serial.print(".");
        wifiRetry++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" ✅ Connected!");
        Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println(" ❌ Failed!");
        Serial.println("[WiFi] Continuing without WiFi (Blynk disabled)");
    }
    
    // Initialize Blynk
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[Blynk] Connecting...");
        Blynk.config(BLYNK_AUTH_TOKEN);
        Blynk.connect(5000);  // 5 second timeout
        
        if (Blynk.connected()) {
            Serial.println("[Blynk] ✅ Connected!");
        } else {
            Serial.println("[Blynk] ❌ Connection failed, will retry");
        }
    }
    
    // Initialize BLE
    Serial.println("[BLE] Initializing...");
    BLEDevice::init("ESP32-Receiver");
    
    // Start scanning
    startBLEScan();
    
    Serial.println("\n=============================================");
    Serial.println("  Setup Complete - System Running");
    Serial.println("=============================================\n");
}

// ================= LOOP =======================
void loop() {
    // Run Blynk
    if (WiFi.status() == WL_CONNECTED) {
        Blynk.run();
        updateBlynkStatus();
    }
    
    // BLE Connection handling
    if (doConnect && !bleConnected) {
        doConnect = false;
        
        if (connectToServer()) {
            Serial.println("[Main] BLE connection successful");
        } else {
            Serial.println("[Main] BLE connection failed, will rescan");
            delay(RECONNECT_DELAY);
            startBLEScan();
        }
    }
    
    // Check BLE connection status
    if (millis() - lastBLECheck >= BLE_SCAN_INTERVAL) {
        lastBLECheck = millis();
        
        // Jika tidak terhubung dan tidak sedang scan, mulai scan
        if (!bleConnected && !scanning) {
            if (client != nullptr && !client->isConnected()) {
                Serial.println("[Main] BLE disconnected, rescanning...");
                startBLEScan();
            } else if (client == nullptr) {
                startBLEScan();
            }
        }
    }
    
    // Small delay untuk stabilitas
    delay(10);
}