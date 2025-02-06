#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define SENSOR_PUMP_1 12
#define SENSOR_PUMP_2 13
#define SENSOR_PUMP_3 15
#define SENSOR_PUMP_4 14

const char* ssid = "rUSSIA_IS_A_TERRORIST_STATE";
const char* password = "9rfq53eutx4cyi86zvki";
const char* serverUrl = "http://192.168.1.65:3000/api/post";

// Global struct for pump settings
struct PumpSettings {
    int min_humidity;
    int min_interval;
    bool water_now;
    int min_sensor_reading;
    int max_sensor_reading;
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
    Serial.println("🔌 Fully disabling Wi-Fi...");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100); // Short delay to allow shutdown

    Serial.println("♻️ Re-enabling ADC2...");
    adcAttachPin(SENSOR_PUMP_1);
    adcAttachPin(SENSOR_PUMP_2);
    adcAttachPin(SENSOR_PUMP_3);
    adcAttachPin(SENSOR_PUMP_4);
}

void setup() {
    Serial.begin(115200);
    Serial.println("🚀 ESP32 Moisture Monitoring System Starting...");

    // Set default pump settings
    for (int i = 0; i < 4; i++) {
        pumps[i].min_humidity = 0;
        pumps[i].min_interval = 10000;
        pumps[i].water_now = false;
        pumps[i].min_sensor_reading = 0;
        pumps[i].max_sensor_reading = 4095;
    }

    disableWiFiAndRestoreADC();
}

void loop() {
    Serial.println("🔍 Reading moisture sensors...");

    // Read sensors
    int moisture[4];
    pinMode(SENSOR_PUMP_1, INPUT);
    moisture[0] = analogRead(SENSOR_PUMP_1);
    pinMode(SENSOR_PUMP_2, INPUT);
    moisture[1] = analogRead(SENSOR_PUMP_2);
    pinMode(SENSOR_PUMP_3, INPUT);
    moisture[2] = analogRead(SENSOR_PUMP_3);
    pinMode(SENSOR_PUMP_4, INPUT);
    moisture[3] = analogRead(SENSOR_PUMP_4);

    Serial.println("🌱 Moisture Readings:");
    Serial.printf(" - Pump 1: %d\n", moisture[0]);
    Serial.printf(" - Pump 2: %d\n", moisture[1]);
    Serial.printf(" - Pump 3: %d\n", moisture[2]);
    Serial.printf(" - Pump 4: %d\n", moisture[3]);

    Serial.println("📡 Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    int attempts = 0;

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts > 20) {  // Timeout
            Serial.println("\n❌ Wi-Fi Connection Failed!");
            return;
        }
    }

    Serial.println("\n✅ Wi-Fi Connected!");

    // Prepare JSON data
    StaticJsonDocument<512> jsonDoc;
    jsonDoc["serial"] = getDeviceSerial();
    
    JsonArray pumpsArray = jsonDoc.createNestedArray("pumps");
    for (int i = 0; i < 4; i++) {
        JsonObject pump = pumpsArray.createNestedObject();
        pump["index"] = i;
        
        JsonObject sensorReading = pump.createNestedObject("currentSensorReading");
        sensorReading["timestamp"] = String(millis()); // Fake timestamp, use actual if needed
        sensorReading["moisture"] = moisture[i];
    }

    String jsonData;
    serializeJson(jsonDoc, jsonData);

    Serial.println("📤 Sending data to server...");
    
    // Send data to the server
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonData);
    
    if (httpResponseCode > 0) {
        Serial.printf("✅ Server Response Code: %d\n", httpResponseCode);
        String response = http.getString();
        Serial.println("🔄 Received Response:");
        Serial.println(response);

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
            }
            Serial.println("✅ Pump settings updated successfully!");
        } else {
            Serial.println("❌ Failed to parse server response.");
        }
    } else {
        Serial.printf("❌ HTTP Request Failed: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    http.end();
    
    // Fully disable Wi-Fi and restore ADC functionality
    disableWiFiAndRestoreADC();
    
    // Delay before next loop
    Serial.println("⏳ Waiting before next cycle...");
    delay(10000);
}