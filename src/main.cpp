#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include "esp_camera.h"  // Camera functions

// Wi-Fi Credentials (CHANGE THESE)
const char* ssid = "rUSSIA_IS_A_TERRORIST_STATE";
const char* password = "9rfq53eutx4cyi86zvki";

// Moisture Sensor & Pump Pins
#define SENSOR_PUMP_1 12
#define SENSOR_PUMP_2 13
#define SENSOR_PUMP_3 15
#define SENSOR_PUMP_4 14

#define RELAY_ACTIVE_LOW true  // Set to false if relay is active-high
#define EEPROM_SIZE 32  // Storage for settings

// Web Server
WebServer server(80);

// Struct for plant settings
struct PlantSettings {
    int minHumidity;
    int minInterval;
    int duration;
} plants[4];

// Sensor Readings
int moistureLevels[4] = {0, 0, 0, 0};
int minSensor[4] = {1000, 800, 800, 1000};  // Min raw ADC value per sensor
int maxSensor[4] = {3000, 1100, 1100, 3000};  // Max raw ADC value per sensor

// Function to map raw ADC value to %
int mapToPercent(int raw, int minVal, int maxVal) {
    return constrain(map(raw, minVal, maxVal, 0, 100), 0, 100);
}

// EEPROM Save & Load
void saveSettings() {
    for (int i = 0; i < 4; i++) {
        EEPROM.write(i * 3, plants[i].minHumidity);
        EEPROM.write(i * 3 + 1, plants[i].minInterval);
        EEPROM.write(i * 3 + 2, plants[i].duration);
    }
    EEPROM.commit();
}

void loadSettings() {
    for (int i = 0; i < 4; i++) {
        plants[i].minHumidity = EEPROM.read(i * 3);
        plants[i].minInterval = EEPROM.read(i * 3 + 1);
        plants[i].duration = EEPROM.read(i * 3 + 2);
    }
}

// Pump Activation Function
void activatePump(int pumpPin, int plantIndex) {
    Serial.print("Watering Plant "); Serial.println(plantIndex + 1);
    pinMode(pumpPin, OUTPUT);
    digitalWrite(pumpPin, RELAY_ACTIVE_LOW ? LOW : HIGH);
    delay(plants[plantIndex].duration * 1000);
    digitalWrite(pumpPin, RELAY_ACTIVE_LOW ? HIGH : LOW);
    pinMode(pumpPin, INPUT);
}

// Handle Web Page Request
void handleRoot() {
    String html = "<html><head><style>";
    html += "body { font-family: Arial; text-align: center; }";
    html += "table { width: 80%; margin: auto; border-collapse: collapse; }";
    html += "td, th { border: 1px solid black; padding: 10px; text-align: center; }";
    html += "</style></head><body>";
    
    html += "<h1>ESP32-CAM Plant Monitor</h1>";
    html += "<img src='/snapshot' width='320'><br>";
    html += "<table><tr><th>Plant</th><th>Soil Humidity</th><th>Min %</th><th>Interval (s)</th><th>Duration (s)</th></tr>";

    for (int i = 0; i < 4; i++) {
        html += "<tr><td>Plant " + String(i + 1) + "</td>";
        html += "<td>" + String(mapToPercent(moistureLevels[i], minSensor[i], maxSensor[i])) + "%</td>";
        html += "<td><input type='number' name='minH" + String(i) + "' value='" + String(plants[i].minHumidity) + "'></td>";
        html += "<td><input type='number' name='interval" + String(i) + "' value='" + String(plants[i].minInterval) + "'></td>";
        html += "<td><input type='number' name='duration" + String(i) + "' value='" + String(plants[i].duration) + "'></td>";
        html += "</tr>";
    }

    html += "</table><br><input type='submit' value='Save'></form></body></html>";
    
    server.send(200, "text/html", html);
}

// Handle Settings Update
void handleSave() {
    for (int i = 0; i < 4; i++) {
        plants[i].minHumidity = server.arg("minH" + String(i)).toInt();
        plants[i].minInterval = server.arg("interval" + String(i)).toInt();
        plants[i].duration = server.arg("duration" + String(i)).toInt();
    }
    saveSettings();
    server.send(200, "text/plain", "Settings saved. <a href='/'>Go back</a>");
}

// Handle Camera Snapshot
void handleSnapshot() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Camera capture failed");
        return;
    }
    server.send(200, "image/jpeg", (const char *)fb->buf);
    esp_camera_fb_return(fb);
}

// Setup Function
void setup() {
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);
    loadSettings();

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");

    server.on("/", handleRoot);
    server.on("/save", handleSave);
    server.on("/snapshot", handleSnapshot);
    server.begin();
}

// Main Loop
void loop() {
    server.handleClient();
    
    // Read Sensors
    for (int i = 0; i < 4; i++) {
        pinMode(SENSOR_PUMP_1 + i, INPUT);
        moistureLevels[i] = analogRead(SENSOR_PUMP_1 + i);
    }

    // Check & Water Plants
    for (int i = 0; i < 4; i++) {
        int humidity = mapToPercent(moistureLevels[i], minSensor[i], maxSensor[i]);
        if (humidity < plants[i].minHumidity) {
            activatePump(SENSOR_PUMP_1 + i, i);
        }
    }

    delay(5000);
}