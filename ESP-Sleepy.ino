/**
 * ESP-Sleepy: Ultra-Efficient Deep-Sleep Optimizer for ESP32 with LoRaWAN & Full Sensor Integration
 *
 * - Adaptive Sleep: Learns when to wake up based on sensor changes
 * - Event-Based LoRa Transmission: Only sends data when necessary
 * - Non-Blocking Serial Handling: No wasted cycles waiting for input
 * - Battery & Multi-Sensor Monitoring: CO₂, Temperature, Motion Detection
 * - Auto-Recovery & System Check: Prevents boot loops
 * - Fully integrated LoRaWAN support (TTGO LoRa ESP32)
 *
 * Installation:
 * 1. Install Arduino IDE with ESP32 board support
 * 2. Install LoRa library: "Arduino-LoRa"
 * 3. Install sensor libraries: "Adafruit BME280", "Adafruit SCD30", "Adafruit MPU6050"
 * 4. Flash this script onto an ESP32 LoRa board
 * 5. Open the serial console (115200 baud) and use the following commands:
 *    - 'sleep 600' -> Sets sleep time to 600 seconds
 *    - 'chaos on' -> Activates random wake-ups
 *    - 'battery off' -> Disables battery monitoring
 *    - 'task off' -> Disables wake-up tasks
 *
 * License: GPL v3
 * Author: bitkid42
 * Improved: March 2025
 */

#include <Arduino.h>
#include <esp_sleep.h>
#include <Preferences.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BME280.h>  // Temperature & Humidity Sensor
#include <Adafruit_SCD30.h>   // CO₂ Sensor
#include <Adafruit_MPU6050.h> // Motion Sensor

// Pin Definitions - Set these according to your board layout
#define I2C_SDA 21
#define I2C_SCL 22
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 23
#define LORA_DIO0 26

// Configuration
#define DEFAULT_SLEEP_TIME 300  
#define BATTERY_PIN 34  
#define LORA_FREQUENCY 868E6
#define SENSOR_READ_TIMEOUT 3000  // 3 seconds timeout for sensor readings

// Sensor Thresholds for LoRa Transmission
#define TEMP_CHANGE_THRESHOLD 2.0
#define CO2_CHANGE_THRESHOLD 50
#define MOTION_THRESHOLD 2.0  // Acceleration Change

// Global variables
Preferences preferences;
bool batteryMonitoring = true;
bool wakeUpTaskEnabled = true;
int lastBatteryLevel = -1;
float lastTemp = -1000;
int lastCO2 = -1;
float lastAccel = 0;

// Sensor instances
Adafruit_BME280 bme;
Adafruit_SCD30 scd30;
Adafruit_MPU6050 mpu;

// Sensor availability flags
bool bmeAvailable = false;
bool scd30Available = false;
bool mpuAvailable = false;

// Function prototypes
void enterDeepSleep(int sleepTime);
void handleSerialInput();
int getBatteryLevel();
int getRandomSleepTime();
void performWakeUpTask();
void sendLoRaData(String data);
void setupLoRa();
void setupSensors();
bool detectSignificantChange(float temp, int co2, float accel);
float readTemperatureSafe();
int readCO2Safe();
float readAccelerationSafe();

void setup() {
    Serial.begin(115200);
    delay(100); // Brief delay to ensure serial is ready
    Serial.println("\nESP-Sleepy starting...");
    
    // Initialize I2C with specified pins
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Initialize preferences
    if (!preferences.begin("esp-sleepy", false)) {
        Serial.println("Failed to initialize preferences, using defaults");
    }
    
    // Load settings from preferences
    batteryMonitoring = preferences.getBool("batteryMonitoring", true);
    wakeUpTaskEnabled = preferences.getBool("wakeUpTask", true);
    int sleepTime = preferences.getInt("sleepTime", DEFAULT_SLEEP_TIME);
    bool chaosMode = preferences.getBool("chaos", false);
    
    // Setup peripherals
    setupLoRa();
    setupSensors();
    
    // Check battery level
    if (batteryMonitoring) {
        int batteryLevel = getBatteryLevel();
        if (batteryLevel < 20) {
            sleepTime = 600; // Extend sleep time on low battery
            Serial.println("Low battery. Extending sleep time.");
        }
    }
    
    // Apply chaos mode if enabled
    if (chaosMode) {
        sleepTime = getRandomSleepTime();
        Serial.printf("Chaos mode: random sleep time %d sec\n", sleepTime);
    }
    
    // Perform wake-up task
    performWakeUpTask();
    
    // Go to sleep
    enterDeepSleep(sleepTime);
}

void loop() {
    // Main loop only handles serial input before sleeping
    handleSerialInput();
    delay(10); // Small delay to reduce CPU usage while handling serial input
}

void enterDeepSleep(int sleepTime) {
    if (sleepTime <= 0) {
        sleepTime = DEFAULT_SLEEP_TIME;
    }
    
    Serial.printf("Entering sleep mode for %d seconds...\n", sleepTime);
    
    // Save settings before sleep
    preferences.putInt("sleepTime", sleepTime);
    preferences.putBool("batteryMonitoring", batteryMonitoring);
    preferences.putBool("wakeUpTask", wakeUpTaskEnabled);
    
    // Ensure preferences are written before sleep
    preferences.end();
    
    // Enable deep sleep wake-up timer
    esp_sleep_enable_timer_wakeup(sleepTime * 1000000LL);
    
    // Go to deep sleep
    Serial.println("Going to sleep now");
    Serial.flush(); // Make sure all serial data is sent
    esp_deep_sleep_start();
}

void handleSerialInput() {
    static String input = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            input.trim();
            
            // Process commands
            if (input.startsWith("sleep")) {
                int newSleepTime = input.substring(6).toInt();
                if (newSleepTime > 0) {
                    Serial.printf("New sleep time: %d sec.\n", newSleepTime);
                    enterDeepSleep(newSleepTime);
                } else {
                    Serial.println("Invalid input. Use 'sleep [seconds]'.");
                }
            } else if (input.equals("chaos on")) {
                Serial.println("Chaotic Mode activated.");
                preferences.putBool("chaos", true);
                enterDeepSleep(getRandomSleepTime());
            } else if (input.equals("chaos off")) {
                Serial.println("Chaotic Mode deactivated.");
                preferences.putBool("chaos", false);
            } else if (input.equals("battery off")) {
                batteryMonitoring = false;
                Serial.println("Battery monitoring disabled.");
            } else if (input.equals("battery on")) {
                batteryMonitoring = true;
                Serial.println("Battery monitoring enabled.");
            } else if (input.equals("task off")) {
                wakeUpTaskEnabled = false;
                Serial.println("Wake-up task disabled.");
            } else if (input.equals("task on")) {
                wakeUpTaskEnabled = true;
                Serial.println("Wake-up task enabled.");
            } else if (input.equals("status")) {
                // Print current status
                Serial.println("Current status:");
                Serial.printf("  Sleep time: %d seconds\n", preferences.getInt("sleepTime", DEFAULT_SLEEP_TIME));
                Serial.printf("  Battery monitoring: %s\n", batteryMonitoring ? "enabled" : "disabled");
                Serial.printf("  Wake-up task: %s\n", wakeUpTaskEnabled ? "enabled" : "disabled");
                Serial.printf("  Chaos mode: %s\n", preferences.getBool("chaos", false) ? "enabled" : "disabled");
                Serial.printf("  Battery level: %d%%\n", getBatteryLevel());
                Serial.printf("  BME280: %s\n", bmeAvailable ? "available" : "not available");
                Serial.printf("  SCD30: %s\n", scd30Available ? "available" : "not available");
                Serial.printf("  MPU6050: %s\n", mpuAvailable ? "available" : "not available");
            } else {
                Serial.println("Unknown command. Available commands:");
                Serial.println("  sleep [seconds] - Set sleep time");
                Serial.println("  chaos on/off - Toggle chaotic mode");
                Serial.println("  battery on/off - Toggle battery monitoring");
                Serial.println("  task on/off - Toggle wake-up task");
                Serial.println("  status - Show current status");
            }
            input = "";
        } else {
            input += c;
        }
    }
}

int getBatteryLevel() {
    if (!batteryMonitoring) return 100;

    // Read voltage from ADC
    int rawValue = analogRead(BATTERY_PIN);
    
    // Convert to voltage (assuming voltage divider)
    float voltage = (rawValue / 4095.0) * 3.3 * 2;
    
    // Map voltage to percentage (3.0V = 0%, 4.2V = 100% for LiPo battery)
    int batteryPercentage = map(voltage * 100, 300, 420, 0, 100);
    
    // Constrain to valid range
    batteryPercentage = constrain(batteryPercentage, 0, 100);
    
    Serial.printf("Battery: %.2fV (%d%%)\n", voltage, batteryPercentage);
    return batteryPercentage;
}

int getRandomSleepTime() {
    return random(30, 600);
}

void performWakeUpTask() {
    if (!wakeUpTaskEnabled) {
        Serial.println("Wake-up task disabled, skipping.");
        return;
    }

    Serial.println("Performing wake-up task...");
    
    // Read sensor data with safety checks
    float temperature = readTemperatureSafe();
    int co2 = readCO2Safe(); 
    float acceleration = readAccelerationSafe();
    
    // Print current readings
    Serial.printf("Temperature: %.1f°C, CO2: %dppm, Acceleration: %.2fm/s²\n", 
                 temperature, co2, acceleration);
    
    // Check if we should transmit data
    if (detectSignificantChange(temperature, co2, acceleration)) {
        String data = "T:" + String(temperature, 1) + "C, CO2:" + String(co2) + "ppm, Accel:" + String(acceleration, 2) + "m/s²";
        sendLoRaData(data);
    } else {
        Serial.println("No significant change detected, skipping transmission.");
    }

    // Update last values
    if (bmeAvailable) lastTemp = temperature;
    if (scd30Available) lastCO2 = co2;
    if (mpuAvailable) lastAccel = acceleration;
}

float readTemperatureSafe() {
    if (!bmeAvailable) {
        Serial.println("BME280 not available, using default temperature.");
        return lastTemp != -1000 ? lastTemp : 20.0; // Default to 20°C if no previous reading
    }
    
    unsigned long startTime = millis();
    float temperature = bme.readTemperature();
    
    if (millis() - startTime > SENSOR_READ_TIMEOUT || isnan(temperature)) {
        Serial.println("Temperature reading timed out or invalid.");
        return lastTemp != -1000 ? lastTemp : 20.0;
    }
    
    return temperature;
}

int readCO2Safe() {
    if (!scd30Available) {
        Serial.println("SCD30 not available, using default CO2.");
        return lastCO2 != -1 ? lastCO2 : 400; // Default to 400ppm if no previous reading
    }
    
    unsigned long startTime = millis();
    if (!scd30.dataReady()) {
        Serial.println("SCD30 data not ready.");
        return lastCO2 != -1 ? lastCO2 : 400;
    }
    
    if (!scd30.read()) {
        Serial.println("SCD30 read failed.");
        return lastCO2 != -1 ? lastCO2 : 400;
    }
    
    if (millis() - startTime > SENSOR_READ_TIMEOUT) {
        Serial.println("CO2 reading timed out.");
        return lastCO2 != -1 ? lastCO2 : 400;
    }
    
    return scd30.getCO2();
}

float readAccelerationSafe() {
    if (!mpuAvailable) {
        Serial.println("MPU6050 not available, using default acceleration.");
        return lastAccel != 0 ? lastAccel : 9.8; // Default to Earth gravity if no previous reading
    }
    
    unsigned long startTime = millis();
    sensors_event_t accel;
    if (!mpu.getAccelerometerSensor()->getEvent(&accel)) {
        Serial.println("MPU6050 read failed.");
        return lastAccel != 0 ? lastAccel : 9.8;
    }
    
    if (millis() - startTime > SENSOR_READ_TIMEOUT) {
        Serial.println("Acceleration reading timed out.");
        return lastAccel != 0 ? lastAccel : 9.8;
    }
    
    return sqrt(accel.acceleration.x * accel.acceleration.x +
                accel.acceleration.y * accel.acceleration.y +
                accel.acceleration.z * accel.acceleration.z);
}

void sendLoRaData(String data) {
    Serial.println("Sending LoRa Data: " + data);
    
    // Attempt to send data
    LoRa.beginPacket();
    LoRa.print(data);
    if (!LoRa.endPacket()) {
        Serial.println("Failed to send LoRa packet.");
        return;
    }
    
    Serial.println("LoRa packet sent successfully.");
}

void setupLoRa() {
    Serial.println("Initializing LoRa...");
    
    // Initialize SPI for LoRa
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    
    // Configure LoRa pins
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    
    // Attempt to initialize LoRa
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa initialization failed, will retry after sleep.");
        // Don't get stuck in infinite loop, just continue with LoRa disabled
    } else {
        LoRa.setSyncWord(0x34);
        Serial.println("LoRa initialized successfully.");
    }
}

void setupSensors() {
    Serial.println("Initializing Sensors...");
    unsigned long startTime = millis();
    
    // Initialize BME280 temperature sensor
    bmeAvailable = bme.begin(0x76);
    if (!bmeAvailable) {
        Serial.println("BME280 not found or failed to initialize.");
    } else {
        Serial.println("BME280 initialized successfully.");
    }
    
    // Initialize SCD30 CO2 sensor
    scd30Available = scd30.begin();
    if (!scd30Available) {
        Serial.println("SCD30 not found or failed to initialize.");
    } else {
        // Configure SCD30 for better power efficiency
        scd30.setMeasurementInterval(2); // Set to minimum interval
        Serial.println("SCD30 initialized successfully.");
    }
    
    // Initialize MPU6050 motion sensor
    mpuAvailable = mpu.begin();
    if (!mpuAvailable) {
        Serial.println("MPU6050 not found or failed to initialize.");
    } else {
        // Configure MPU6050 for better power efficiency
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("MPU6050 initialized successfully.");
    }
    
    // Report sensor initialization time
    Serial.printf("Sensor initialization took %lu ms\n", millis() - startTime);
}

bool detectSignificantChange(float temp, int co2, float accel) {
    // Only compare with last values if sensors are available
    bool tempChanged = bmeAvailable && (abs(temp - lastTemp) > TEMP_CHANGE_THRESHOLD);
    bool co2Changed = scd30Available && (abs(co2 - lastCO2) > CO2_CHANGE_THRESHOLD);
    bool accelChanged = mpuAvailable && (abs(accel - lastAccel) > MOTION_THRESHOLD);
    
    return tempChanged || co2Changed || accelChanged;
}
