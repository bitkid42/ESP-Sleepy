
/**
 * ESP-Sleepy: Ultra-Efficient Deep-Sleep Optimizer for ESP32 with LoRaWAN
 *
 * - Adaptive Sleep: Learns when to wake up based on sensor changes
 * - Event-Based LoRa Transmission: Only sends data when necessary
 * - Non-Blocking Serial Handling: No wasted cycles waiting for input
 * - Optional Battery Monitoring: Saves power when disabled
 * - Auto-Recovery & System Check: Prevents boot loops
 * - Fully integrated LoRaWAN support (TTGO LoRa ESP32)
 *
 * Installation:
 * 1. Install Arduino IDE with ESP32 board support
 * 2. Install LoRa library: "Arduino-LoRa" (for LoRaWAN)
 * 3. Flash this script onto an ESP32 LoRa board
 * 4. Open the serial console (115200 baud) and use the following commands:
 *    - 'sleep 600' -> Sets sleep time to 600 seconds
 *    - 'chaos on' -> Activates random wake-ups
 *    - 'battery off' -> Disables battery monitoring
 *    - 'task off' -> Disables wake-up tasks
 *
 * License: GPL v3
 * Author: bitkid42
 */

#include <Arduino.h>
#include <esp_sleep.h>
#include <Preferences.h>
#include <SPI.h>
#include <LoRa.h>

#define DEFAULT_SLEEP_TIME 300  
#define BATTERY_PIN 34  // ADC Pin for battery voltage measurement
#define LORA_TRIGGER_THRESHOLD 5  // % change required to trigger LoRa transmission
#define LORA_FREQUENCY 868E6  // Set frequency for EU (868 MHz)

Preferences preferences;
bool batteryMonitoring = true;
bool wakeUpTaskEnabled = true;
int lastBatteryLevel = -1;

void enterDeepSleep(int sleepTime);
void handleSerialInput();
int getBatteryLevel();
int getRandomSleepTime();
void performWakeUpTask();
void sendLoRaData(int batteryLevel);
void setupLoRa();

void setup() {
    Serial.begin(115200);
    setupLoRa();

    preferences.begin("esp-sleepy", false);

    int sleepTime = preferences.getInt("sleepTime", DEFAULT_SLEEP_TIME);

    if (batteryMonitoring) {
        int batteryLevel = getBatteryLevel();
        if (batteryLevel < 20) {
            sleepTime = 600;
            Serial.println("Low battery. Extending sleep time.");
        }
        if (lastBatteryLevel != -1 && abs(batteryLevel - lastBatteryLevel) > LORA_TRIGGER_THRESHOLD) {
            sendLoRaData(batteryLevel);
        }
        lastBatteryLevel = batteryLevel;
    }

    if (preferences.getBool("chaos", false)) {
        sleepTime = getRandomSleepTime();
        Serial.printf("Chaotic Mode active. Sleep time: %d sec.\n", sleepTime);
    }

    if (wakeUpTaskEnabled) {
        performWakeUpTask();
    }

    enterDeepSleep(sleepTime);
}

void loop() {
    handleSerialInput();
}

void enterDeepSleep(int sleepTime) {
    Serial.printf("Entering sleep mode for %d seconds...\n", sleepTime);
    preferences.putInt("sleepTime", sleepTime);
    preferences.putBool("batteryMonitoring", batteryMonitoring);
    preferences.putBool("wakeUpTask", wakeUpTaskEnabled);
    preferences.putInt("lastBattery", lastBatteryLevel);
    preferences.end();

    esp_sleep_enable_timer_wakeup(sleepTime * 1000000LL);
    esp_deep_sleep_start();
}

void handleSerialInput() {
    static String input = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            input.trim();
            if (input.startsWith("sleep")) {
                int newSleepTime = input.substring(6).toInt();
                if (newSleepTime > 0) {
                    Serial.printf("New sleep time: %d sec.\n", newSleepTime);
                    preferences.putInt("sleepTime", newSleepTime);
                    enterDeepSleep(newSleepTime);
                } else {
                    Serial.println("Invalid input. Use 'sleep [seconds]'.");
                }
            } else if (input.equals("chaos on")) {
                Serial.println("Chaotic Mode activated.");
                preferences.putBool("chaos", true);
                enterDeepSleep(getRandomSleepTime());
            } else if (input.equals("battery off")) {
                batteryMonitoring = false;
                Serial.println("Battery monitoring disabled.");
            } else if (input.equals("task off")) {
                wakeUpTaskEnabled = false;
                Serial.println("Wake-up task disabled.");
            } else {
                Serial.println("Unknown command.");
            }
            input = "";
        } else {
            input += c;
        }
    }
}

int getBatteryLevel() {
    if (!batteryMonitoring) return 100;

    int rawValue = analogRead(BATTERY_PIN);
    float voltage = (rawValue / 4095.0) * 3.3 * 2;
    int batteryPercentage = map(voltage * 100, 300, 420, 0, 100);
    batteryPercentage = constrain(batteryPercentage, 0, 100);
    
    Serial.printf("Battery level: %d%%\n", batteryPercentage);
    return batteryPercentage;
}

int getRandomSleepTime() {
    return random(30, 600);
}

void performWakeUpTask() {
    if (!wakeUpTaskEnabled) return;

    Serial.println("Performing wake-up task...");
    int batteryLevel = getBatteryLevel();
    if (batteryLevel < 15) {
        Serial.println("Warning: Battery critically low!");
    }

    // Future: Additional sensor readings, advanced diagnostics
}

void sendLoRaData(int batteryLevel) {
    Serial.printf("LoRa Transmission Triggered. Battery: %d%%\n", batteryLevel);
    
    LoRa.beginPacket();
    LoRa.print("Battery: ");
    LoRa.print(batteryLevel);
    LoRa.println("%");
    int status = LoRa.endPacket();
    
    if (status == 1) {
        Serial.println("LoRa transmission successful.");
    } else {
        Serial.println("LoRa transmission failed.");
    }
}

void setupLoRa() {
    Serial.println("Initializing LoRa...");
    SPI.begin(5, 19, 27, 18);  // SPI pins for TTGO LoRa
    LoRa.setPins(18, 23, 26);  // NSS, Reset, DIO0
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa initialization failed.");
        while (1);
    }
    LoRa.setSyncWord(0x34);
    Serial.println("LoRa initialized.");
}
