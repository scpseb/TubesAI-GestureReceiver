/*
 * ESP32 BLE Receiver with FreeRTOS + Blynk Integration
 * 
 * Menerima kode gesture dari BLE Sender (XIAO ESP32S3):
 * - Kode 0: Matikan semua (Relay OFF, Fan OFF)
 * - Kode 1: Nyalakan Relay (Lampu ON)
 * - Kode 2: Nyalakan Fan
 * 
 * FreeRTOS Tasks:
 * - BLE Task: Handle scanning dan koneksi BLE (Core 0)
 * - Blynk Task: Handle komunikasi Blynk (Core 0)
 * - Control Task: Handle relay dan fan control (Core 1)
 * 
 * Blynk Virtual Pins:
 * - V0: Status Relay (0/1) - Output ke app
 * - V1: Status Fan (0/1) - Output ke app
 * - V2: Manual control Relay - Input dari app
 * - V3: Manual control Fan - Input dari app
 * - V4: Last gesture code - Output ke app
 * - V5: BLE connection status - Output ke app
 */

#include <Arduino.h>

// ================= BLYNK CONFIG (HARUS DI ATAS) =================
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

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ================= WIFI CONFIG =================
const char* ssid     = "Ash";      // Ganti dengan SSID WiFi
const char* password = "jadibegini";  // Ganti dengan password WiFi

// ================= PIN CONFIG =================
#define RELAY_PIN       2     // Relay (lampu)
#define FAN_PIN         16    // Fan

// Pin behavior
#define RELAY_ACTIVE_LOW  false   // Set true jika relay active low
#define FAN_ACTIVE_HIGH   true    // Fan ON saat pin HIGH

// ================= BLE UUID ===================
#define SERVICE_UUID        "0001046d-9d7a-4cc4-8cd9-e385bc31104d"
#define CHARACTERISTIC_UUID "babc1b75-f950-4e74-8248-5e984a9efb09"

// ================= BLYNK VIRTUAL PINS =========
#define VPIN_RELAY_STATUS   V0    // Status relay (output)
#define VPIN_FAN_STATUS     V1    // Status fan (output)
#define VPIN_RELAY_CONTROL  V2    // Kontrol relay dari app (input)
#define VPIN_FAN_CONTROL    V3    // Kontrol fan dari app (input)
#define VPIN_GESTURE_CODE   V4    // Kode gesture terakhir (output)
#define VPIN_BLE_STATUS     V5    // Status koneksi BLE (output)

// ================= RTOS CONFIG ================
#define BLE_TASK_PRIORITY       1
#define BLYNK_TASK_PRIORITY     1
#define CONTROL_TASK_PRIORITY   2

// Stack sizes - INCREASED to prevent overflow
#define BLE_TASK_STACK_SIZE     4096    // Was 4096
#define BLYNK_TASK_STACK_SIZE   8192    // Was 4096  
#define CONTROL_TASK_STACK_SIZE 8192    // Was 2048

#define CONTROL_QUEUE_SIZE  10

// ================= CONTROL COMMAND TYPES ======
typedef enum {
    CMD_GESTURE,        // Dari BLE gesture
    CMD_BLYNK_RELAY,    // Dari Blynk app - relay
    CMD_BLYNK_FAN,      // Dari Blynk app - fan
} command_source_t;

typedef struct {
    command_source_t source;
    uint8_t value;
} control_command_t;

// ================= GLOBAL STATE ===============
static bool relayState = false;
static bool fanState = false;
static uint8_t lastGestureCode = 255;

// BLE states
static BLEAdvertisedDevice* myDevice = nullptr;
static BLERemoteCharacteristic* remoteChar = nullptr;
static BLEClient* client = nullptr;
static volatile bool bleConnected = false;
static volatile bool doConnect = false;
static volatile bool scanning = false;

// FreeRTOS handles
TaskHandle_t bleTaskHandle = NULL;
TaskHandle_t blynkTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;
QueueHandle_t controlQueue = NULL;
SemaphoreHandle_t stateMutex = NULL;

// ================= FUNCTION DECLARATIONS ======
void relayWrite(bool on);
void fanWrite(bool on);
void handleGestureCode(uint8_t code);
bool connectToServer();
void startBLEScan();

void bleTask(void *pvParameters);
void blynkTask(void *pvParameters);
void controlTask(void *pvParameters);

// ================= RELAY & FAN CONTROL ========
void relayWrite(bool on) {
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        relayState = on;
        
        if (RELAY_ACTIVE_LOW) {
            digitalWrite(RELAY_PIN, on ? LOW : HIGH);
        } else {
            digitalWrite(RELAY_PIN, on ? HIGH : LOW);
        }
        
        Serial.printf("[Relay] %s (pin=%d)\n", on ? "ON" : "OFF", digitalRead(RELAY_PIN));
        xSemaphoreGive(stateMutex);
    }
}

void fanWrite(bool on) {
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        fanState = on;
        
        if (FAN_ACTIVE_HIGH) {
            digitalWrite(FAN_PIN, on ? HIGH : LOW);
        } else {
            digitalWrite(FAN_PIN, on ? LOW : HIGH);
        }
        
        Serial.printf("[Fan] %s (pin=%d)\n", on ? "ON" : "OFF", digitalRead(FAN_PIN));
        xSemaphoreGive(stateMutex);
    }
}

// ================= GESTURE HANDLER ============
void handleGestureCode(uint8_t code) {
    Serial.printf("\n[Gesture] Processing code: %d\n", code);
    
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        lastGestureCode = code;
        xSemaphoreGive(stateMutex);
    }
    
    switch (code) {
        case 0:
            relayWrite(false);
            fanWrite(false);
            Serial.println("[Action] ALL OFF");
            break;
            
        case 1:
            relayWrite(true);
            fanWrite(false);
            Serial.println("[Action] RELAY ON, FAN OFF");
            break;
            
        case 2:
            relayWrite(false);
            fanWrite(true);
            Serial.println("[Action] RELAY OFF, FAN ON");
            break;
            
        default:
            Serial.printf("[Action] Unknown code: %d\n", code);
            break;
    }
}

// ================= BLYNK CALLBACKS ============
BLYNK_WRITE(VPIN_RELAY_CONTROL) {
    int value = param.asInt();
    Serial.printf("[Blynk] Relay control: %d\n", value);
    
    control_command_t cmd;
    cmd.source = CMD_BLYNK_RELAY;
    cmd.value = (uint8_t)value;
    
    if (controlQueue != NULL) {
        xQueueSend(controlQueue, &cmd, pdMS_TO_TICKS(10));
    }
}

BLYNK_WRITE(VPIN_FAN_CONTROL) {
    int value = param.asInt();
    Serial.printf("[Blynk] Fan control: %d\n", value);
    
    control_command_t cmd;
    cmd.source = CMD_BLYNK_FAN;
    cmd.value = (uint8_t)value;
    
    if (controlQueue != NULL) {
        xQueueSend(controlQueue, &cmd, pdMS_TO_TICKS(10));
    }
}

BLYNK_CONNECTED() {
    Serial.println("[Blynk] Connected!");
    
    bool relay, fan;
    uint8_t gesture;
    
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        relay = relayState;
        fan = fanState;
        gesture = lastGestureCode;
        xSemaphoreGive(stateMutex);
    }
    
    Blynk.virtualWrite(VPIN_RELAY_STATUS, relay ? 1 : 0);
    Blynk.virtualWrite(VPIN_FAN_STATUS, fan ? 1 : 0);
    Blynk.virtualWrite(VPIN_BLE_STATUS, bleConnected ? 1 : 0);
    
    if (gesture != 255) {
        Blynk.virtualWrite(VPIN_GESTURE_CODE, gesture);
    }
    
    Blynk.syncVirtual(VPIN_RELAY_CONTROL);
    Blynk.syncVirtual(VPIN_FAN_CONTROL);
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
    Serial.printf("[BLE] Received code: %d\n", code);
    
    control_command_t cmd;
    cmd.source = CMD_GESTURE;
    cmd.value = code;
    
    if (controlQueue != NULL) {
        xQueueSend(controlQueue, &cmd, pdMS_TO_TICKS(10));
    }
}

// ================= BLE SCAN CALLBACK ==========
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (advertisedDevice.haveName() || advertisedDevice.haveServiceUUID()) {
            Serial.printf("[BLE Scan] Found: %s\n", advertisedDevice.toString().c_str());
        }
        
        bool hasCorrectUUID = advertisedDevice.haveServiceUUID() &&
                              advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID));
        
        bool hasCorrectName = advertisedDevice.haveName() &&
                              (strcmp(advertisedDevice.getName().c_str(), "Gesture-XIAO") == 0);
        
        if (hasCorrectUUID || hasCorrectName) {
            Serial.println("[BLE Scan] ✅ Found Gesture-XIAO!");
            
            if (myDevice != nullptr) {
                delete myDevice;
            }
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            scanning = false;
            
            BLEDevice::getScan()->stop();
        }
    }
};

// ================= BLE CLIENT CALLBACK ========
class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        Serial.println("[BLE] Client connected");
    }

    void onDisconnect(BLEClient* pclient) override {
        bleConnected = false;
        Serial.println("[BLE] ❌ Disconnected");
    }
};

// ================= BLE CONNECT ================
bool connectToServer() {
    if (myDevice == nullptr) return false;
    
    Serial.printf("[BLE] Connecting to: %s\n", myDevice->getAddress().toString().c_str());
    
    if (client == nullptr) {
        client = BLEDevice::createClient();
        client->setClientCallbacks(new MyClientCallback());
    }
    
    if (!client->connect(myDevice)) {
        Serial.println("[BLE] ❌ Connection failed");
        return false;
    }
    
    BLERemoteService* remoteService = client->getService(BLEUUID(SERVICE_UUID));
    if (remoteService == nullptr) {
        Serial.println("[BLE] ❌ Service not found");
        client->disconnect();
        return false;
    }
    
    remoteChar = remoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
    if (remoteChar == nullptr) {
        Serial.println("[BLE] ❌ Characteristic not found");
        client->disconnect();
        return false;
    }
    
    if (remoteChar->canNotify()) {
        remoteChar->registerForNotify(notifyCallback);
    } else {
        client->disconnect();
        return false;
    }
    
    bleConnected = true;
    Serial.println("[BLE] ✅ CONNECTED");
    return true;
}

// ================= BLE SCAN START =============
void startBLEScan() {
    if (scanning) return;
    
    Serial.println("[BLE] Starting scan...");
    scanning = true;
    doConnect = false;
    
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    
    // Non-blocking scan with callback
    pBLEScan->start(10, [](BLEScanResults results) {
        Serial.printf("[BLE Scan] Complete, found %d devices\n", results.getCount());
        scanning = false;
    }, false);
}

// ================= BLE TASK ===================
void bleTask(void *pvParameters) {
    Serial.println("[BLE Task] Started");
    
    // Small delay before init
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    BLEDevice::init("ESP32-Receiver");
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    startBLEScan();
    
    while (1) {
        if (doConnect && !bleConnected) {
            doConnect = false;
            
            if (!connectToServer()) {
                vTaskDelay(pdMS_TO_TICKS(3000));
                if (!scanning) {
                    startBLEScan();
                }
            }
        }
        
        // Check connection and rescan if needed
        if (!bleConnected && !scanning && !doConnect) {
            static unsigned long lastScanAttempt = 0;
            if (millis() - lastScanAttempt > 5000) {
                lastScanAttempt = millis();
                
                if (client != nullptr) {
                    if (!client->isConnected()) {
                        Serial.println("[BLE Task] Disconnected, rescanning...");
                        startBLEScan();
                    }
                } else {
                    startBLEScan();
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ================= BLYNK TASK =================
void blynkTask(void *pvParameters) {
    Serial.println("[Blynk Task] Started");
    
    // Wait for WiFi with timeout
    int waitCount = 0;
    while (WiFi.status() != WL_CONNECTED && waitCount < 100) {
        vTaskDelay(pdMS_TO_TICKS(100));
        waitCount++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[Blynk Task] WiFi not connected, task will retry");
    }
    
    Blynk.config(BLYNK_AUTH_TOKEN);
    
    unsigned long lastSync = 0;
    unsigned long lastConnectAttempt = 0;
    
    while (1) {
        if (WiFi.status() == WL_CONNECTED) {
            // Try to connect if not connected (with rate limiting)
            if (!Blynk.connected()) {
                if (millis() - lastConnectAttempt > 10000) {  // Try every 10 seconds
                    lastConnectAttempt = millis();
                    Serial.println("[Blynk Task] Connecting...");
                    Blynk.connect(3000);  // Shorter timeout
                }
            }
            
            // Run Blynk if connected
            if (Blynk.connected()) {
                Blynk.run();
                
                // Periodic sync every 2 seconds
                if (millis() - lastSync >= 2000) {
                    lastSync = millis();
                    
                    bool relay = false, fan = false;
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        relay = relayState;
                        fan = fanState;
                        xSemaphoreGive(stateMutex);
                    }
                    
                    Blynk.virtualWrite(VPIN_RELAY_STATUS, relay ? 1 : 0);
                    Blynk.virtualWrite(VPIN_FAN_STATUS, fan ? 1 : 0);
                    Blynk.virtualWrite(VPIN_BLE_STATUS, bleConnected ? 1 : 0);
                }
            }
        } else {
            // WiFi disconnected
            static unsigned long lastReconnect = 0;
            if (millis() - lastReconnect > 30000) {  // Try every 30 seconds
                lastReconnect = millis();
                Serial.println("[Blynk Task] WiFi lost, reconnecting...");
                WiFi.reconnect();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Increased delay for stability
    }
}

// ================= CONTROL TASK ===============
void controlTask(void *pvParameters) {
    Serial.println("[Control Task] Started");
    
    control_command_t cmd;
    
    while (1) {
        if (xQueueReceive(controlQueue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.printf("[Control] src=%d, val=%d\n", cmd.source, cmd.value);
            
            switch (cmd.source) {
                case CMD_GESTURE:
                    handleGestureCode(cmd.value);
                    break;
                    
                case CMD_BLYNK_RELAY:
                    relayWrite(cmd.value == 1);
                    break;
                    
                case CMD_BLYNK_FAN:
                    fanWrite(cmd.value == 1);
                    break;
            }
            
            // Update Blynk
            if (Blynk.connected()) {
                bool relay, fan;
                uint8_t gesture;
                
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    relay = relayState;
                    fan = fanState;
                    gesture = lastGestureCode;
                    xSemaphoreGive(stateMutex);
                }
                
                Blynk.virtualWrite(VPIN_RELAY_STATUS, relay ? 1 : 0);
                Blynk.virtualWrite(VPIN_FAN_STATUS, fan ? 1 : 0);
                
                if (cmd.source == CMD_GESTURE) {
                    Blynk.virtualWrite(VPIN_GESTURE_CODE, gesture);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================= SETUP ======================
void setup() {
    Serial.begin(115200);
    delay(2000);  // Longer delay for stability
    
    Serial.println("\n=============================================");
    Serial.println("  ESP32 BLE Receiver + Blynk + FreeRTOS");
    Serial.println("=============================================\n");
    
    // Pin setup
    if (RELAY_ACTIVE_LOW) {
        digitalWrite(RELAY_PIN, HIGH);
    } else {
        digitalWrite(RELAY_PIN, LOW);
    }
    digitalWrite(FAN_PIN, LOW);
    
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    
    if (RELAY_ACTIVE_LOW) {
        digitalWrite(RELAY_PIN, HIGH);
    } else {
        digitalWrite(RELAY_PIN, LOW);
    }
    digitalWrite(FAN_PIN, LOW);
    
    Serial.printf("[Setup] RELAY=%d, FAN=%d\n", digitalRead(RELAY_PIN), digitalRead(FAN_PIN));
    Serial.printf("[Setup] Free heap: %d bytes\n", ESP.getFreeHeap());
    
    // Create RTOS objects
    stateMutex = xSemaphoreCreateMutex();
    controlQueue = xQueueCreate(CONTROL_QUEUE_SIZE, sizeof(control_command_t));
    
    if (stateMutex == NULL || controlQueue == NULL) {
        Serial.println("ERR: Failed to create RTOS objects!");
        while (1) { delay(1000); }
    }
    
    // WiFi connect
    Serial.printf("[WiFi] Connecting to %s", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        Serial.print(".");
        retry++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf(" ✅ IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println(" ❌ Failed!");
    }
    
    Serial.printf("[Setup] Free heap after WiFi: %d bytes\n", ESP.getFreeHeap());
    
    // Create tasks with error checking
    Serial.println("[Setup] Creating tasks...");
    
    BaseType_t result;
    
    // Control Task first (highest priority, on Core 1)
    result = xTaskCreatePinnedToCore(
        controlTask, 
        "Control", 
        CONTROL_TASK_STACK_SIZE, 
        NULL, 
        CONTROL_TASK_PRIORITY, 
        &controlTaskHandle, 
        1
    );
    if (result != pdPASS) {
        Serial.println("ERR: Control task failed!");
    } else {
        Serial.println("[Setup] Control task created");
    }
    
    delay(100);
    
    // Blynk Task (Core 0)
    result = xTaskCreatePinnedToCore(
        blynkTask, 
        "Blynk", 
        BLYNK_TASK_STACK_SIZE, 
        NULL, 
        BLYNK_TASK_PRIORITY, 
        &blynkTaskHandle, 
        0
    );
    if (result != pdPASS) {
        Serial.println("ERR: Blynk task failed!");
    } else {
        Serial.println("[Setup] Blynk task created");
    }
    
    delay(100);
    
    // BLE Task (Core 0) - created last
    result = xTaskCreatePinnedToCore(
        bleTask, 
        "BLE", 
        BLE_TASK_STACK_SIZE, 
        NULL, 
        BLE_TASK_PRIORITY, 
        &bleTaskHandle, 
        0
    );
    if (result != pdPASS) {
        Serial.println("ERR: BLE task failed!");
    } else {
        Serial.println("[Setup] BLE task created");
    }
    
    Serial.printf("[Setup] Free heap after tasks: %d bytes\n", ESP.getFreeHeap());
    Serial.println("\n[Setup] Complete - System Running\n");
}

// ================= LOOP =======================
void loop() {
    // Print stack high water mark periodically for debugging
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 10000) {
        lastCheck = millis();
        
        Serial.println("\n--- Stack Monitor ---");
        if (bleTaskHandle != NULL) {
            Serial.printf("BLE Task: %d words free\n", uxTaskGetStackHighWaterMark(bleTaskHandle));
        }
        if (blynkTaskHandle != NULL) {
            Serial.printf("Blynk Task: %d words free\n", uxTaskGetStackHighWaterMark(blynkTaskHandle));
        }
        if (controlTaskHandle != NULL) {
            Serial.printf("Control Task: %d words free\n", uxTaskGetStackHighWaterMark(controlTaskHandle));
        }
        Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
        Serial.println("--------------------\n");
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
}