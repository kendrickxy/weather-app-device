#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_SHTC3.h>
#include <SparkFunBQ27441.h> // Added for BQ27441
#include <ArduinoJson.h>
#include <Preferences.h>
#include "secret.h" 
#include "time.h"   
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "driver/rtc_io.h"

// I2C Pins for SHTC3
#define SHTC3_I2C_SDA 13 
#define SHTC3_I2C_SCL 11 

// I2C Pins for BQ27441
#define BQ_I2C_SDA 34
#define BQ_I2C_SCL 33

// Sleep related
#define uS_TO_S_FACTOR 1000000ULL     
#define SLEEP_DURATION (5 * 60) // 5 minutes

// Battery capacity (mAh) - Set this to your battery's design capacity
const unsigned int BATTERY_CAPACITY = 240; // Adjust as needed

Preferences preferences; 

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
WebServer server(80); 

RTC_DATA_ATTR int bootCount = 0;
bool isAPMode = false; 

// NTP data
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600; 
const int daylightOffset_sec = 3600; 

// WiFi Configuration
String wifiSSID = WIFI_SSID; 
String wifiPassword = WIFI_PASS; 
const char *apSSID = "TempSensorConfig"; 
// const char *apPassword = AP_PASSWORD;

// Firebase configuration
const String FIREBASE_HOST = FIREBASE_HOST_S;
const String FIREBASE_AUTH = FIREBASE_AUTH_S;
const String FIREBASE_PATH = "/sensorData.json"; 

// --- BQ27441 Variables ---
// These will hold the latest battery readings
unsigned int batterySOC = 0;
unsigned int batteryVoltage = 0;
int batteryCurrent = 0; // Can be negative for discharge

// --- Function Declarations for Web Server ---
void handleRoot();
void handleConfigure();
void handleStatus();
void handleRestart();

unsigned long getCurrentUnixTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time from NTP");
      return 0; // Indicates failure
  }
  return mktime(&timeinfo); // Convert to milliseconds
}

// --- BQ27441 Functions ---
bool setupBQ27441() {
  Serial.println("Configuring I2C for BQ27441 (SDA:34, SCL:33)...");
  Wire.end(); // End any previous I2C bus usage on Wire
  delay(100);
  Wire.begin(BQ_I2C_SDA, BQ_I2C_SCL); 
  Wire.setClock(100000); 
  delay(100); 

  Serial.println("Attempting to initialize BQ27441 LiPo Fuel Gauge...");
  if (!lipo.begin()) { 
    Serial.println("ERROR: Unable to communicate with BQ27441.");
    Serial.println("  - Check wiring, pull-ups, power, and battery connection.");
    return false; // Indicate failure
  }
  Serial.println("BQ27441 detected and initialized!");
  if (lipo.setCapacity(BATTERY_CAPACITY)) {
    Serial.print("BQ27441: Battery capacity set to "); Serial.print(BATTERY_CAPACITY); Serial.println(" mAh.");
  } else {
    Serial.println("BQ27441: Error setting battery capacity.");
  }
  return true; // Indicate success
}

void readAndUpdateBatteryStats() {
  if (!lipo.begin()) { // Check if BQ27441 is connected (based on successful begin)
    Serial.println("BQ27441 not connected, cannot read battery stats.");
    batterySOC = 0; // Reset a global or indicate error
    batteryVoltage = 0;
    batteryCurrent = 0;
    return;
  }

  batterySOC = lipo.soc();
  batteryVoltage = lipo.voltage();
  batteryCurrent = lipo.current(AVG); // Use AVG for average current

  Serial.print("Battery Stats Updated: SoC="); Serial.print(batterySOC);
  Serial.print("%, V="); Serial.print(batteryVoltage);
  Serial.print("mV, I="); Serial.print(batteryCurrent); Serial.println("mA");

  // Handle potential error readings
  if (batterySOC == 65535 || batteryVoltage == 65535) {
      Serial.println("Warning: Read invalid battery stats (65535), communication issue?");
      // You might want to set them to a specific error value like 0 or -1
      batterySOC = 0; 
      batteryVoltage = 0;
      batteryCurrent = 0;
  }
}


// --- WiFi and Server Functions ---
void startAPMode() {
  Serial.println("Starting Access Point (AP) mode for configuration...");
  WiFi.disconnect(true); 
  delay(100);
  WiFi.softAP(apSSID, AP_PASSWORD); 
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  isAPMode = true;

  server.on("/", HTTP_GET, handleRoot);
  server.on("/configure", HTTP_POST, handleConfigure);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/restart", HTTP_GET, handleRestart);
  
  server.begin();
  Serial.println("HTTP server started for AP configuration.");
}

void loadWiFiCredentials() {
  preferences.begin("wifi-config", false); 
  wifiSSID = preferences.getString("ssid", WIFI_SSID); 
  wifiPassword = preferences.getString("pass", WIFI_PASS); 
  preferences.end();
  Serial.println("Loaded WiFi credentials.");
  Serial.print("SSID: "); Serial.println(wifiSSID);
}

void saveWiFiCredentials(String newSSID, String newPass) {
  preferences.begin("wifi-config", false);
  preferences.putString("ssid", newSSID);
  preferences.putString("pass", newPass);
  preferences.end();
  wifiSSID = newSSID;
  wifiPassword = newPass;
  Serial.println("WiFi credentials saved.");
}

bool attemptWiFiConnection() {
  if (wifiSSID == "" || wifiSSID == "your_wifi_ssid") { 
    Serial.println("WiFi SSID not configured. Starting AP mode.");
    startAPMode();
    return false;
  }

  Serial.print("Attempting to connect to WiFi: ");
  Serial.println(wifiSSID);
  WiFi.mode(WIFI_STA); 
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { 
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nSuccessfully connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Configuring time using NTP...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    String currentTime = String(getCurrentUnixTimestamp()); 
    Serial.print("Current time: "); Serial.println(currentTime);
    isAPMode = false;
    return true;
  } else {
    Serial.println("\nFailed to connect to WiFi.");
    startAPMode(); 
    return false;
  }
}

// --- sendToFirebase ---
void sendToFirebase(float temperature, float humidity, unsigned int battSOC, unsigned int battVoltage, int battCurrent) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Cannot send to Firebase - WiFi not connected.");
    return;
  }

  HTTPClient http;
  String url = "https://" + FIREBASE_HOST + FIREBASE_PATH + "?auth=" + FIREBASE_AUTH;

  JsonDocument doc; 
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["battery_soc"] = battSOC;
  // doc["battery_voltage_mv"] = battVoltage;
  // doc["battery_current_ma"] = battCurrent;
  doc["timestamp"] = getCurrentUnixTimestamp(); 
  doc["deviceId"] = WiFi.macAddress(); 

  String payload;
  serializeJson(doc, payload);

  Serial.println("Sending data to Firebase...");
  Serial.println(payload); 

  http.begin(url); 
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.POST(payload);

  if (httpCode > 0) {
    String response = http.getString();
    Serial.printf("Firebase POST response code: %d\n", httpCode);
    Serial.println("Firebase response: " + response);
  } else {
    Serial.printf("Firebase POST failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.print("Wakeup caused by: ");
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Timer"); break;
    default: Serial.printf("Other reason (%d) - Likely first boot\n", wakeup_reason); break;
  }
}

// --- Web Server Handlers ---
void handleRoot() {
  String html = "<html><head><title>Sensor Config</title>";
  html += "<style>body{font-family:Arial,sans-serif;margin:20px;background:#f4f4f4;color:#333;}h1{color:#005c99;}.container{background:#fff;padding:20px;border-radius:8px;box-shadow:0 0 10px rgba(0,0,0,0.1);max-width:500px;margin:auto;}label{display:block;margin-top:10px;margin-bottom:5px;font-weight:bold;}input[type='text'],input[type='password']{width:calc(100% - 22px);padding:10px;margin-bottom:10px;border:1px solid #ddd;border-radius:4px;}button{background-color:#005c99;color:white;padding:10px 15px;border:none;border-radius:4px;cursor:pointer;font-size:16px;}button:hover{background-color:#004477;}a{color:#005c99;text-decoration:none;}a:hover{text-decoration:underline;}.status-table{width:100%;margin-top:20px;border-collapse:collapse;}.status-table th,.status-table td{text-align:left;padding:8px;border-bottom:1px solid #ddd;}</style>";
  html += "</head><body><div class='container'>";
  html += "<h1>ESP32 Sensor Configuration</h1>";

  if (isAPMode) {
    html += "<p><strong>Device is in Configuration Mode.</strong><br>Connect this device to a WiFi network.</p>";
  } else {
    html += "<p>Device is attempting to connect to: <strong>" + wifiSSID + "</strong></p>";
  }

  html += "<form action='/configure' method='POST'>";
  html += "<label for='ssid'>WiFi SSID:</label><input type='text' id='ssid' name='ssid' value='" + wifiSSID + "' required>";
  html += "<label for='password'>WiFi Password:</label><input type='password' id='password' name='password' value='" + wifiPassword + "' required>";
  html += "<button type='submit'>Save and Connect</button>";
  html += "</form>";
  html += "<p style='margin-top:20px;'><a href='/status'>View System Status</a></p>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

void handleConfigure() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    String newSSID = server.arg("ssid");
    String newPass = server.arg("password");
    Serial.println("New WiFi config submitted:");
    Serial.print("SSID: "); Serial.println(newSSID);
    
    saveWiFiCredentials(newSSID, newPass);

    String html = "<html><head><title>Config Saved</title><meta http-equiv='refresh' content='5;url=/'></head><body><div style='font-family:Arial;text-align:center;margin-top:50px;'>";
    html += "<h1>Configuration Saved!</h1>";
    html += "<p>Attempting to connect to '<strong>" + newSSID + "</strong>'.<br>The device will restart. If connection fails, it will revert to AP mode.</p>";
    html += "<p>You will be redirected shortly. If not, please connect to '" + String(apSSID) + "' to reconfigure or check device status.</p>";
    html += "</div></body></html>";
    server.send(200, "text/html", html);

    delay(1000);
    ESP.restart(); 
  } else {
    server.send(400, "text/plain", "Bad Request: SSID and password required.");
  }
}

void handleRestart() {
    server.send(200, "text/html", "<html><head><title>Restarting</title><meta http-equiv='refresh' content='3;url=/'></head><body><h1>Device is restarting...</h1></body></html>");
    delay(1000);
    ESP.restart();
}

void handleStatus() { // Updated to show battery info
  String html = "<html><head><title>System Status</title>";
  html += "<style>body{font-family:Arial,sans-serif;margin:20px;background:#f4f4f4;color:#333;}h1{color:#005c99;}.container{background:#fff;padding:20px;border-radius:8px;box-shadow:0 0 10px rgba(0,0,0,0.1);max-width:600px;margin:auto;}table{width:100%;border-collapse:collapse;margin-top:20px;}th,td{padding:10px;border:1px solid #ddd;text-align:left;}th{background-color:#e9ecef;}</style>";
  html += "</head><body><div class='container'>";
  html += "<h1>System Status</h1>";
  html += "<table>";
  html += "<tr><th>Parameter</th><th>Value</th></tr>";
  html += "<tr><td>Boot Count</td><td>" + String(bootCount) + "</td></tr>";
  html += "<tr><td>Current Mode</td><td>" + String(isAPMode ? "Access Point (AP)" : "Station (STA)") + "</td></tr>";
  html += "<tr><td>Target WiFi SSID</td><td>" + wifiSSID + "</td></tr>";
  html += "<tr><td>WiFi Connection</td><td>" + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + "</td></tr>";
  if (WiFi.status() == WL_CONNECTED && !isAPMode) {
    html += "<tr><td>IP Address</td><td>" + WiFi.localIP().toString() + "</td></tr>";
  } else if (isAPMode) {
    html += "<tr><td>AP IP Address</td><td>" + WiFi.softAPIP().toString() + "</td></tr>";
  }
  html += "<tr><td>MAC Address</td><td>" + WiFi.macAddress() + "</td></tr>";
  
  // SHTC3 Sensor status
  Wire.end(); // End BQ I2C
  delay(50);
  Wire.begin(SHTC3_I2C_SDA, SHTC3_I2C_SCL); // Init for SHTC3
  Wire.setClock(100000);
  delay(50);
  Wire.beginTransmission(0x70); 
  bool shtc3Found = (Wire.endTransmission() == 0);
  html += "<tr><td>SHTC3 Sensor</td><td>" + String(shtc3Found ? "Detected" : "Not Detected") + "</td></tr>";
  if(shtc3Found){
    sensors_event_t h, t;
    if(shtc3.getEvent(&h, &t)){ 
        if(!isnan(t.temperature) && !isnan(h.relative_humidity)){
            html += "<tr><td>Last Temperature</td><td>" + String(t.temperature, 2) + " *C</td></tr>";
            html += "<tr><td>Last Humidity</td><td>" + String(h.relative_humidity, 2) + " %</td></tr>";
        } else {
            html += "<tr><td>SHTC3 Reading</td><td>Error (NaN)</td></tr>";
        }
    } else {
         html += "<tr><td>SHTC3 Reading</td><td>Error (getEvent)</td></tr>";
    }
  }

  // BQ27441 Battery Status
  Wire.end(); // End SHTC3 I2C
  delay(50);
  Wire.begin(BQ_I2C_SDA, BQ_I2C_SCL); // Init for BQ
  Wire.setClock(100000);
  delay(50);
  bool bqFound = lipo.begin(); // Quick check if BQ is still responding
  html += "<tr><td>BQ27441 Fuel Gauge</td><td>" + String(bqFound ? "Detected" : "Not Detected") + "</td></tr>";
  if (bqFound) {
    // Use the globally updated values if available, or read fresh
    // For status page, reading fresh is fine.
    html += "<tr><td>Battery SoC</td><td>" + String(lipo.soc()) + " %</td></tr>";
    html += "<tr><td>Battery Voltage</td><td>" + String(lipo.voltage()) + " mV</td></tr>";
    html += "<tr><td>Battery Current</td><td>" + String(lipo.current(AVG)) + " mA</td></tr>";
  }


  html += "<tr><td>NTP Time Sync</td><td>" + String(getCurrentUnixTimestamp()) + "</td></tr>";
  html += "<tr><td>Sleep Duration</td><td>" + String(SLEEP_DURATION) + " seconds</td></tr>";
  html += "</table>";
  html += "<p style='margin-top:20px;'><a href='/'>Back to Main Page</a></p>";
  html += "<p><a href='/restart'>Restart Device</a></p>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}


void setup() {
  rtc_gpio_deinit(GPIO_NUM_11); // SHTC3_I2C_SCL
  rtc_gpio_deinit(GPIO_NUM_13); // SHTC3_I2C_SDA

  Serial.begin(115200);
  delay(2000); 

  Serial.printf("\n--- Boot count: %d ---\n", bootCount);
  print_wakeup_reason();
  bootCount++;

  // --- Initialize BQ27441 (Battery Monitor) ---
  bool bqOk = setupBQ27441(); // This configures Wire for BQ pins
  if (bqOk) {
    readAndUpdateBatteryStats(); // Get initial battery readings
  } else {
    Serial.println("Continuing without BQ27441 battery data.");
    // Set default error values if BQ failed
    batterySOC = 0; batteryVoltage = 0; batteryCurrent = 0; 
  }
  
  // --- Initialize SHTC3 (Temperature/Humidity Sensor) ---
  // Wire object needs to be re-initialized for the SHTC3 pins
  pinMode(SHTC3_I2C_SDA, INPUT_PULLUP);
  pinMode(SHTC3_I2C_SCL, INPUT_PULLUP);
  delay(100);

  Wire.end(); // End previous I2C configuration (from BQ27441)
  delay(100); 
  
  bool shtc3Ok = false;
  Serial.println("Initializing I2C bus for SHTC3...");
  if (Wire.begin(SHTC3_I2C_SDA, SHTC3_I2C_SCL)) { 
    Wire.setClock(100000); // Set I2C clock speed
    Serial.println("I2C bus initialized for SHTC3.");
    Serial.println("Initializing SHTC3 sensor...");
    if (shtc3.begin()) { 
      Serial.println("SHTC3 sensor initialized successfully.");
      shtc3Ok = true;
    } else {
      Serial.println("SHTC3 sensor initialization FAILED!");
    }
  } else {
    Serial.println("I2C bus initialization FAILED for SHTC3!");
  }

  loadWiFiCredentials(); 

  if (attemptWiFiConnection()) { 
    float temperature = -777.0;
    float humidity = -777.0;

    if (shtc3Ok) {
      // Re-check Wire config for SHTC3 before reading, just in case
      Wire.end(); delay(50); Wire.begin(SHTC3_I2C_SDA, SHTC3_I2C_SCL); Wire.setClock(100000); delay(50);

      sensors_event_t humidity_event, temp_event;
      if (shtc3.getEvent(&humidity_event, &temp_event)) {
        if (!isnan(temp_event.temperature) && !isnan(humidity_event.relative_humidity)) {
          temperature = temp_event.temperature;
          humidity = humidity_event.relative_humidity;
          Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" *C");
          Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
        } else {
          Serial.println("Failed to read valid data from SHTC3 sensor!");
        }
      } else {
        Serial.println("Error reading from SHTC3 sensor post-WiFi connect!");
      }
    } else {
      Serial.println("SHTC3 Sensor not OK, cannot get temp/humidity data.");
    }

    // Update battery stats again before sending (Wire is currently set for SHTC3)
    if (bqOk) {
        Wire.end(); delay(50); Wire.begin(BQ_I2C_SDA, BQ_I2C_SCL); Wire.setClock(100000); delay(50);
        readAndUpdateBatteryStats();
    }

    sendToFirebase(temperature, humidity, batterySOC, batteryVoltage, batteryCurrent);
    
    Serial.println("Configuring RTC Clock to INTERNAL RC for deep sleep...");
    rtc_clk_slow_freq_set(RTC_SLOW_FREQ_RTC);

    Serial.printf("Enabling timer wakeup for %d seconds...\n", SLEEP_DURATION);
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION * uS_TO_S_FACTOR);

    Serial.println("De-initializing I2C bus before sleep...");
    Wire.end();
    pinMode(SHTC3_I2C_SDA, INPUT); 
    pinMode(SHTC3_I2C_SCL, INPUT);
    pinMode(BQ_I2C_SDA, INPUT); 
    pinMode(BQ_I2C_SCL, INPUT);
    
    Serial.println("Closing WiFi connection before sleep...");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF); 
    delay(100);

    Serial.println("Entering deep sleep now.");
    Serial.flush();
    esp_deep_sleep_start();
  } else {
    // WiFi connection failed, AP mode started
    Serial.println("Device is in AP Mode. Configure WiFi at http://" + WiFi.softAPIP().toString());
  }
}

void loop() {
  if (isAPMode) {
    server.handleClient(); 

    static unsigned long lastBatteryRead = 0;
    if (millis() - lastBatteryRead > 5000) { // Every 5 seconds
      Wire.end(); delay(50); Wire.begin(BQ_I2C_SDA, BQ_I2C_SCL); Wire.setClock(100000); delay(50);
      readAndUpdateBatteryStats();
      lastBatteryRead = millis();
    }
    delay(10); 
  } else {
    Serial.println("Error: In loop() but not in AP Mode and not sleeping. Restarting.");
    delay(1000);
    ESP.restart();
  }
}