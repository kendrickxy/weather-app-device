# ESP32-S2 Environmental Sensor & Battery Monitor with Firebase Logging

This project implements a low-power environmental sensor node using an ESP32-S2 microcontroller. It reads temperature and humidity from an SHTC3 sensor, monitors LiPo battery voltage, current, and state of charge (SoC) using a BQ27441 fuel gauge, and periodically sends this data to a Firebase Realtime Database. The device utilizes deep sleep to conserve power and features a WiFi configuration mode via a web portal if it cannot connect to a known network.

## Features

* **Environmental Sensing:** Reads temperature and humidity using an Adafruit SHTC3 sensor.
* **Battery Monitoring:**
    * Measures battery voltage, current (average), and State of Charge (SoC) using a SparkFun BQ27441 LiPo Fuel Gauge.
    * Configurable battery capacity.
* **WiFi Connectivity:**
    * Connects to a configured WiFi network to send data.
    * Stores WiFi credentials persistently in Non-Volatile Storage (NVS) using the `Preferences` library.
* **Access Point (AP) Configuration Mode:**
    * If WiFi credentials are not set or connection fails, the device starts an AP.
    * Users can connect to this AP (SSID: `TempSensorConfig`) and configure WiFi SSID and password via a simple web portal.
* **Data Logging:** Sends sensor data (temperature, humidity) and battery metrics (SoC, voltage, current) along with a timestamp and device ID to a specified Firebase Realtime Database.
* **Low Power Operation:**
    * Utilizes ESP32-S2 deep sleep functionality to minimize power consumption between readings.
    * Currently configured for a 5-minute sleep interval (adjustable).
    * Uses the ESP32-S2's internal RTC for sleep timing, ensuring stability.
* **NTP Time Synchronization:** Fetches current time from an NTP server to timestamp data accurately.

## Hardware Requirements

* **Microcontroller:** ESP32-S2 based board.
* **Temperature/Humidity Sensor:** Adafruit SHTC3 (or compatible SHTC3 module).
    * **SCL Pin:** Connected to ESP32-S2 GPIO 11
    * **SDA Pin:** Connected to ESP32-S2 GPIO 13
* **LiPo Fuel Gauge:** SparkFun BQ27441 LiPo Fuel Gauge (or compatible BQ27441 module).
    * **SCL Pin:** Connected to ESP32-S2 GPIO 33
    * **SDA Pin:** Connected to ESP32-S2 GPIO 34
    * **Important:** Ensure the VSS (GND) pin of the BQ27441 is properly connected to ground.
* **LiPo Battery:** Single-cell (1S) Lithium Polymer battery compatible with the BQ27441.
* **Pull-up Resistors:** External pull-up resistors (e.g., 4.7kΩ) are **required** for both I2C buses:
    * One set for SHTC3 (GPIO 11 to 3.3V, GPIO 13 to 3.3V).
    * One set for BQ27441 (GPIO 33 to 3.3V, GPIO 34 to 3.3V).
* **Standard Development Board Components:** USB cable for programming/power, etc.

## Software & Libraries

* [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/) for ESP32 development.
* ESP32 Board Support Package.
* **Libraries (can be installed via Arduino Library Manager):**
    * `WiFi.h` (standard ESP32 library)
    * `WebServer.h` (standard ESP32 library)
    * `HTTPClient.h` (standard ESP32 library)
    * `Wire.h` (standard I2C library)
    * `Adafruit_SHTC3` by Adafruit
    * `SparkFun_BQ27441_LiPo_Fuel_Gauge_Arduino_Library` by SparkFun Electronics (ensure you have the version compatible with the `lipo.begin()` signature used, typically installs as "SparkFun BQ27441 LiPo Fuel Gauge")
    * `ArduinoJson` by Benoit Blanchon (Version 6.x recommended)
    * `Preferences.h` (standard ESP32 library for NVS)

## Setup & Configuration

1.  **Hardware Connections:**
    * Connect the SHTC3 sensor to the ESP32-S2 using the I2C pins specified above (SCL: GPIO 11, SDA: GPIO 13). Ensure 3.3V and GND are also connected.
    * Connect the BQ27441 fuel gauge to the ESP32-S2 using its I2C pins (SCL: GPIO 33, SDA: GPIO 34). Ensure 3.3V, GND (VSS), and the LiPo battery are correctly connected to the BQ27441 module.
    * **Crucially, add external 4.7kΩ pull-up resistors to 3.3V for all four I2C lines (SHTC3_SCL, SHTC3_SDA, BQ_SCL, BQ_SDA).**

2.  **`secret.h` File:**
    Create a file named `secret.h` in the same directory as your `main.cpp` (or in your Arduino sketch folder). This file stores your sensitive credentials and is excluded from version control by `.gitignore` (recommended).
    It should contain the following definitions:

    ```cpp
    #ifndef SECRET_H
    #define SECRET_H

    // WiFi Credentials (used as default if nothing is stored in NVS, or for first boot)
    #define WIFI_SSID "your_default_wifi_ssid"
    #define WIFI_PASS "your_default_wifi_password"

    // Access Point Password (for the configuration portal)
    #define AP_PASSWORD "your_ap_setup_password" // Choose a secure password

    // Firebase Configuration
    #define FIREBASE_HOST_S "YOUR_PROJECT_ID.firebaseio.com" // Replace with your Firebase Host URL
    #define FIREBASE_AUTH_S "YOUR_FIREBASE_DATABASE_SECRET_OR_AUTH_TOKEN" // Replace with your Firebase Auth Token or Database Secret

    #endif // SECRET_H
    ```

3.  **Battery Capacity:**
    In `main.cpp`, adjust the `BATTERY_CAPACITY` constant to match the design capacity of your LiPo battery in mAh:
    ```cpp
    const unsigned int BATTERY_CAPACITY = 500; // e.g., for a 500mAh battery
    ```

4.  **Sleep Duration & Timezone:**
    * `SLEEP_DURATION` in `main.cpp` controls how long the device sleeps (in seconds).
    * `gmtOffset_sec` and `daylightOffset_sec` can be adjusted for your local timezone for NTP.

5.  **Upload Code:**
    Compile and upload the `main.cpp` sketch to your ESP32-S2 board.

## How to Use

1.  **First Boot / No WiFi Configured:**
    * On the first boot, or if the device cannot connect to the stored WiFi credentials, it will start an Access Point (AP).
    * The AP SSID will be `TempSensorConfig`.
    * Connect to this AP using the password defined in `AP_PASSWORD` (from `secret.h`).
    * Open a web browser and navigate to `http://192.168.4.1` (the default IP for ESP32 soft AP).
    * You will see a configuration page where you can enter the SSID and password for your local WiFi network.
    * Click "Save and Connect". The device will save the credentials and attempt to restart and connect.

2.  **Normal Operation:**
    * The device wakes up from deep sleep.
    * It attempts to connect to the configured WiFi network.
    * If successful, it synchronizes time with an NTP server.
    * It initializes and reads data from the SHTC3 sensor (temperature, humidity).
    * It initializes and reads data from the BQ27441 fuel gauge (SoC, voltage, current).
    * It sends the collected data to your Firebase database.
    * It then goes back into deep sleep for the duration defined by `SLEEP_DURATION`.

3.  **Accessing Device Status (AP Mode):**
    * If the device is in AP mode, you can navigate to `http://192.168.4.1/status` to view system status, including sensor detection and basic battery info.

## Notes & Troubleshooting

* **BQ27441 Accuracy:** The BQ27441 uses an Impedance Track™ algorithm that learns your battery's characteristics over time. For best accuracy, it's recommended to allow the battery to go through a few full charge/discharge cycles while connected to the gauge. The SoC readings will become more reliable with use.
* **I2C Communication:**
    * The project uses the ESP32's default `Wire` I2C object, re-initializing it sequentially for the SHTC3 and then the BQ27441 as they are on different pins.
    * If you encounter issues with I2C device detection, double-check your wiring, ensure the correct VSS/GND connections, and verify the presence and value of external pull-up resistors.
* **Deep Sleep Current:** The BQ27441 itself, along with other components, will have a quiescent current draw. The 5-minute sleep interval provides good relaxation periods for the BQ27441 to take OCV readings and improve its battery model.
* **Firebase Rules:** Ensure your Firebase Realtime Database rules are configured to allow writes to the specified `FIREBASE_PATH`.

## Future Enhancements (Optional)

* Implement OTA (Over-The-Air) updates for remote firmware flashing.
* Add more detailed error reporting to Firebase.
* Further power optimization for even longer battery life.
* Integrate a display to show readings locally.

---