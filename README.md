# ESP-Sleepy

## The Deep-Sleep Optimizer for ESP32 + LoRaWAN + Sensors

**Minimal. Smart. Efficient.**  
ESP-Sleepy helps your ESP32 devices sleep better, wake up smarter, and only transmit when it really matters.  
It’s like caffeine withdrawal – but for your microcontroller.

## Features
✅ **Adaptive Deep-Sleep:** Learns when to wake up, instead of just running on a dumb timer.  
✅ **Event-Based LoRa Transmission:** No unnecessary data spam – only sends when real changes occur.  
✅ **Non-Blocking Serial Input:** Your ESP32 won’t hang around waiting for commands like a lost puppy.  
✅ **Battery & Sensor Monitoring:** CO₂, Temperature, Motion – all in one.  
✅ **Auto-Recovery & System Check:** If something goes wrong, ESP-Sleepy knows what to do.  

## Supported Hardware
- **ESP32 LoRa Boards (TTGO, Heltec, etc.)**
- **BME280 (Temp & Humidity)**
- **SCD30 (CO₂ Sensor)**
- **MPU6050 (Motion Detection)**
- **Anything that runs on I²C or SPI if you feel lucky**

## Installation
1. Install **Arduino IDE** with ESP32 board support.  
2. Install these libraries:
   ```sh
   Arduino-LoRa
   Adafruit BME280
   Adafruit SCD30
   Adafruit MPU6050
   ```
3. Flash `ESP-Sleepy.ino` onto your ESP32.  
4. Open Serial Monitor (115200 baud) and configure it:  
   ```sh
   sleep 600     # Set sleep time in seconds  
   chaos on      # Enable random wake-ups  
   battery off   # Disable battery monitoring  
   task off      # Disable wake-up tasks  
   ```

## Why ESP-Sleepy?
Because life is short and battery power is even shorter.  
Save energy. Save transmissions. Save yourself from debugging bad sleep timers.

## License
Released under **GPL v3** – because Open Source should stay Open Source.  

---  

💡 **Got improvements? Open a pull request. Found a bug? Blame yourself, then open an issue.**  
