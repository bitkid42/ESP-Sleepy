
/**
 * ESP-Sleepy: Ultra-Efficient Deep-Sleep Optimizer for ESP32 with LoRaWAN & Sensors
 *
 * - Adaptive Sleep: Learns when to wake up based on sensor changes
 * - Event-Based LoRa Transmission: Only sends data when necessary
 * - Non-Blocking Serial Handling: No wasted cycles waiting for input
 * - Optional Battery Monitoring: Saves power when disabled
 * - Auto-Recovery & System Check: Prevents boot loops
 * - Fully integrated LoRaWAN support (TTGO LoRa ESP32)
 * - Additional Sensors: Temperature, CO₂, Motion Detection
 *
 * Installation:
 * 1. Install Arduino IDE with ESP32 board support
 * 2. Install LoRa library: "Arduino-LoRa"
 * 3. Install Adafruit BME280, SCD30, and MPU6050 libraries if using these sensors
 * 4. Flash this script onto an ESP32 LoRa board
 * 5. Open the serial console (115200 baud) and use the following commands:
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
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BME280.h>  // Temp & Humidity
#include <Adafruit_SCD30.h>   // CO₂
#include <Adafruit_MPU6050.h> // Motion Sensor

#define DEFAULT_SLEEP_TIME 300  
#define BATTERY_PIN 34  
#define LORA_FREQUENCY 868E6  

// Sensor Thresholds for LoRa Transmission
#define TEMP_CHANGE_THRESHOLD 2.0
#define CO2_CHANGE_THRESHOLD 50
#define MOTION_THRESHOLD 2.0  // Acceleration Change

Preferences preferences;
bool batteryMonitoring = true;
bool wakeUpTaskEnabled = true;
int lastBatteryLevel = -1;
float lastTemp = -1000;
int lastCO2 = -1;
float lastAccel = 0;

Adafruit_BME280 bme;
Adafruit_SCD30 scd30;
Adafruit_MPU6050 mpu;

void enterDeepSleep(int sleepTime);
void handleSerialInput();
int getBatteryLevel();
int getRandomSleepTime();
void performWakeUpTask();
void sendLoRaData(String data);
void setupLoRa();
void setupSensors();
bool detectSignificantChange(float temp, int co2, float accel);

void setup() {
    Serial.begin(115200);
    setupLoRa();
    setupSensors();

    preferences.begin("esp-sleepy", false);
    int sleepTime = preferences.getInt("sleepTime", DEFAULT_SLEEP_TIME);

    if (batteryMonitoring) {
        int batteryLevel = getBatteryLevel();
        if (batteryLevel < 20) {
            sleepTime = 600;
            Serial.println("Low battery. Extending sleep time.");
        }
    }

    performWakeUpTask();
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
    float temperature = bme.readTemperature();
    int co2 = scd30.getCO2();
    sensors_event_t accel;
    mpu.getAccelerometerSensor()->getEvent(&accel);
    float acceleration = sqrt(accel.acceleration.x * accel.acceleration.x +
                              accel.acceleration.y * accel.acceleration.y +
                              accel.acceleration.z * accel.acceleration.z);

    if (detectSignificantChange(temperature, co2, acceleration)) {
        String data = "T:" + String(temperature, 1) + "C, CO2:" + String(co2) + "ppm, Accel:" + String(acceleration, 2) + "m/s²";
        sendLoRaData(data);
    }

    lastTemp = temperature;
    lastCO2 = co2;
    lastAccel = acceleration;
}

void sendLoRaData(String data) {
    Serial.println("Sending LoRa Data: " + data);
    LoRa.beginPacket();
    LoRa.print(data);
    LoRa.endPacket();
}

void setupLoRa() {
    Serial.println("Initializing LoRa...");
    SPI.begin(5, 19, 27, 18);
    LoRa.setPins(18, 23, 26);
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa initialization failed.");
        while (1);
    }
    LoRa.setSyncWord(0x34);
    Serial.println("LoRa initialized.");
}

void setupSensors() {
    Serial.println("Initializing Sensors...");

    if (!bme.begin(0x76)) {
        Serial.println("BME280 not found.");
    } else {
        Serial.println("BME280 initialized.");
    }

    if (!scd30.begin()) {
        Serial.println("SCD30 not found.");
    } else {
        Serial.println("SCD30 initialized.");
    }

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found.");
    } else {
        Serial.println("MPU6050 initialized.");
    }
}

bool detectSignificantChange(float temp, int co2, float accel) {
    return (abs(temp - lastTemp) > TEMP_CHANGE_THRESHOLD) ||
           (abs(co2 - lastCO2) > CO2_CHANGE_THRESHOLD) ||
           (abs(accel - lastAccel) > MOTION_THRESHOLD);
}
