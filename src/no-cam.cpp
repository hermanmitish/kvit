#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

#define SENSOR_PUMP_1 12
#define SENSOR_PUMP_2 13
#define SENSOR_PUMP_3 15
#define SENSOR_PUMP_4 14

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* serverUrl = SERVER_URL;

unsigned long lastRebootTime = 0;
String logBuffer;  // ✅ Global log storage

void logMessage(const String &message) {
    logBuffer += message + "\n";  // ✅ Append log messages
    Serial.println(message);       // ✅ Also print logs to Serial Monitor
}

void checkForReboot() {
    if (millis() - lastRebootTime > 1 * 60 * 60 * 1000) {  // Every 1 hour
        logMessage("🔄 Rebooting ESP32 to free memory...");
        ESP.restart();
    }
}
// Global struct for pump settings
struct PumpSettings {
    int min_humidity;
    int min_interval;
    bool water_now;
    int min_sensor_reading;
    int max_sensor_reading;
    int watering_duration;
} pumps[4];

// Function to get ESP32 serial number
String getDeviceSerial() {
    uint32_t chipId = 0;
    for (int i = 0; i < 17; i += 8) {
        chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xFF) << i;
    }
    return "esp32-" + String(chipId, HEX);
}

void disableWiFiAndRestoreADC() {
    logMessage("🔌 Fully disabling Wi-Fi...");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500); // Short delay to allow shutdown

    logMessage("♻️ Re-enabling ADC2...");
    adcAttachPin(SENSOR_PUMP_1);
    adcAttachPin(SENSOR_PUMP_2);
    adcAttachPin(SENSOR_PUMP_3);
    adcAttachPin(SENSOR_PUMP_4);
}

void activatePump(int pumpIndex, int duration) {
    int pumpPins[4] = {SENSOR_PUMP_1, SENSOR_PUMP_2, SENSOR_PUMP_3, SENSOR_PUMP_4};
    int pumpPin = pumpPins[pumpIndex];

    logMessage("🚰 Activating Pump " + String(pumpIndex) + " for " + String(duration) + " seconds...");
    pinMode(pumpPin, OUTPUT);
    digitalWrite(pumpPin, LOW);  // Activate relay (assuming active LOW)
    
    delay(duration * 1000);  // Run pump

    digitalWrite(pumpPin, HIGH);  // Deactivate relay
    pinMode(pumpPin, INPUT);  // Switch back to sensor mode

    logMessage("✅ Pump " + String(pumpIndex) + " OFF");
}

void setup() {
    Serial.begin(115200);
    logMessage("🚀 ESP32 Humidity Monitoring System Starting...");

    // ✅ **Delayed initialization sequence**
    delay(2000);  // Allow power to stabilize before initialization
    // Set default pump settings
    for (int i = 0; i < 4; i++) {
        pumps[i].min_humidity = 0;
        pumps[i].min_interval = 60*60*24;
        pumps[i].water_now = false;
        pumps[i].min_sensor_reading = 0;
        pumps[i].max_sensor_reading = 4095;
        pumps[i].watering_duration = 5;
    }

    disableWiFiAndRestoreADC();
}

void uploadSensorData(String jsonData){
    logMessage("📤 Sending data to server...");
    
    // Send data to the server
    HTTPClient http;
    http.setTimeout(20000);  // ✅ Increase timeout to 20 seconds (20000ms)
    String path = String(serverUrl) + "/api/post";
    http.begin(path);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonData);
    
    if (httpResponseCode > 0) {
        logMessage("✅ Server Response Code: " + String(httpResponseCode));
        String response = http.getString();
        logMessage("🔄 Received Response:");
        logMessage(response);

        // Parse JSON response
        StaticJsonDocument<512> responseJson;
        DeserializationError error = deserializeJson(responseJson, response);
        if (!error) {
            JsonArray receivedPumps = responseJson["pumps"].as<JsonArray>();
            for (int i = 0; i < receivedPumps.size(); i++) {
                pumps[i].min_humidity = receivedPumps[i]["min_humidity"];
                pumps[i].min_interval = receivedPumps[i]["min_interval"];
                pumps[i].water_now = receivedPumps[i]["water_now"];
                pumps[i].min_sensor_reading = receivedPumps[i]["min_sensor_reading"];
                pumps[i].max_sensor_reading = receivedPumps[i]["max_sensor_reading"];
                pumps[i].watering_duration = receivedPumps[i]["watering_duration"];
            }
            logMessage("✅ Pump settings updated successfully!");
            // ✅ Correct way to extract "timestamp"
            if (responseJson.containsKey("timestamp")) {
                String timestamp = responseJson["timestamp"].as<String>();  // ✅ Extract correctly
                Serial.print("📅 Received Timestamp: ");
                logMessage(timestamp);
            } else {
                logMessage("❌ No 'timestamp' in response JSON.");
            }
        } else {
            logMessage("❌ Failed to parse server response.");
        }
    } else {
        logMessage("❌ HTTP Request Failed: " + String(http.errorToString(httpResponseCode).c_str()));
    }

    http.end();

}

void loop() {
    checkForReboot();

    logMessage("🔍 Waiting before reading humidity sensors...");
    delay(2000); // ✅ **Allow power stabilization before sensor reading**
    // Read sensors
    int humidity[4];
    pinMode(SENSOR_PUMP_1, INPUT);
    humidity[0] = analogRead(SENSOR_PUMP_1);
    pinMode(SENSOR_PUMP_2, INPUT);
    humidity[1] = analogRead(SENSOR_PUMP_2);
    pinMode(SENSOR_PUMP_3, INPUT);
    humidity[2] = analogRead(SENSOR_PUMP_3);
    pinMode(SENSOR_PUMP_4, INPUT);
    humidity[3] = analogRead(SENSOR_PUMP_4);

    logMessage("📡 Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    int attempts = 0;

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts > 20) {  // Timeout
            logMessage("❌ Wi-Fi Connection Failed!");
            return;
        }
    }

    logMessage("✅ Wi-Fi Connected!");
    logMessage("🌱 Humidity Level:");
    // Prepare JSON data
    StaticJsonDocument<512> jsonDoc;
    jsonDoc["serial"] = getDeviceSerial();
    
    JsonArray pumpsArray = jsonDoc.createNestedArray("pumps");
    for (int i = 0; i < 4; i++) {
        JsonObject pump = pumpsArray.createNestedObject();
        pump["index"] = i;
        // // After we manually watered the pump, turn the toggle off
        // pump["water_now"] = false;
        // After we manually or automatically watered the pump, set this toggle to true
        if(pumps[i].water_now){
            pump["watering"] = true;
        }else{
            pump["watering"] = false;
        }
        
        
        JsonObject sensorReading = pump.createNestedObject("currentSensorReading");
        sensorReading["humidityRaw"] = humidity[i];
        float relativeHumidity = max(0.0f, min(1.0f, 
            (float)(pumps[i].max_sensor_reading - humidity[i]) / 
            (float)(pumps[i].max_sensor_reading - pumps[i].min_sensor_reading)
        ));
        sensorReading["humidity"] = relativeHumidity;
        logMessage("Pump " + String(i) + " - Raw: " + String(humidity[i]) + 
            ", Min: " + String(pumps[i].min_sensor_reading) + 
            ", Max: " + String(pumps[i].max_sensor_reading) + 
            ", Relative Humidity: " + String(relativeHumidity * 100) + "%");
    }

    String jsonData;
    // ✅ Add logs to JSON before sending
    jsonDoc["logs"] = logBuffer;
    serializeJson(jsonDoc, jsonData);    
    uploadSensorData(jsonData);
    
    // ✅ Clear log buffer after sending data
    logBuffer = "";

    // Fully disable Wi-Fi and restore ADC functionality
    disableWiFiAndRestoreADC();
    

    // ✅ **Watering Logic**
    unsigned long currentTime = millis() / 1000;
    for (int i = 0; i < 4; i++) {
        if (pumps[i].water_now) {
            logMessage("🚰 Manually watering Pump " + String(i));
            activatePump(i, pumps[i].watering_duration);
        } else {
            logMessage("❌ Pump " + String(i) + ": No watering needed");
        }
        delay(1000); // ✅ **Allow power stabilization before another pump activation**
    }
    logMessage("💾 Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
    // Delay before next loop
    logMessage("⏳ Waiting before next cycle...");
    delay(10000);
}
