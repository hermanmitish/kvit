#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "secrets.h"

#define PWDN_GPIO_NUM     32  // Power down pin (GPIO32 in ESPHome)
#define RESET_GPIO_NUM    -1  // No hardware reset
#define XCLK_GPIO_NUM      0  // External clock (was 21, should be 0)
#define SIOD_GPIO_NUM     26  // I2C SDA
#define SIOC_GPIO_NUM     27  // I2C SCL

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define SENSOR_PUMP_1 12
#define SENSOR_PUMP_2 13
#define SENSOR_PUMP_3 15
#define SENSOR_PUMP_4 14

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* serverUrl = SERVER_URL;
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


void setupCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_VGA;  // Start with QVGA for stability VGA, SVGA, XGA, SXGA, UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;

    Serial.println("🛠 Initializing Camera...");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("❌ Camera Init Failed! Error: 0x%x\n", err);
        return;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID != OV2640_PID) {
        Serial.println("❌ Camera not detected. Check model or connection.");
        return;
    }

    Serial.println("✅ Camera initialized successfully!");
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
    Serial.println("🚀 ESP32 Humidity Monitoring System Starting...");
    setupCamera(); // Initialize camera
    // Set default pump settings
    for (int i = 0; i < 4; i++) {
        pumps[i].min_humidity = 0;
        pumps[i].min_interval = 60*60*24;
        pumps[i].water_now = false;
        pumps[i].min_sensor_reading = 0;
        pumps[i].max_sensor_reading = 4095;
    }

    disableWiFiAndRestoreADC();
}

void uploadImage(String timestamp) {
    Serial.println("📸 Capturing image...");
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("❌ Camera capture failed!");
        return;
    }

    Serial.printf("📤 Uploading image (%d bytes)...\n", fb->len);

    HTTPClient http;
    http.setTimeout(20000);  // ✅ Increase timeout to 20 seconds (20000ms)
    WiFiClient client;
    String path = String(serverUrl) + "/api/upload?serial=" + getDeviceSerial() + "&timestamp=" + timestamp;
    http.begin(client, path); // Adjust endpoint as needed

    http.addHeader("Content-Type", "image/jpeg");

    int httpResponseCode = http.PUT(fb->buf, fb->len);
    if (httpResponseCode > 0) {
        Serial.printf("✅ Image Sent! Server Response: %d\n", httpResponseCode);
    } else {
        Serial.printf("❌ Image Upload Failed: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    esp_camera_fb_return(fb);
    http.end();
}

void uploadSensorData(String jsonData){
    Serial.println("📤 Sending data to server...");
    
    // Send data to the server
    HTTPClient http;
    http.setTimeout(20000);  // ✅ Increase timeout to 20 seconds (20000ms)
    String path = String(serverUrl) + "/api/post";
    http.begin(path);
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
            // ✅ Correct way to extract "timestamp"
            if (responseJson.containsKey("timestamp")) {
                String timestamp = responseJson["timestamp"].as<String>();  // ✅ Extract correctly
                Serial.print("📅 Received Timestamp: ");
                Serial.println(timestamp);
                
                uploadImage(timestamp);  // ✅ Pass timestamp correctly
            } else {
                Serial.println("❌ No 'timestamp' in response JSON.");
            }
        } else {
            Serial.println("❌ Failed to parse server response.");
        }
    } else {
        Serial.printf("❌ HTTP Request Failed: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    http.end();

}

void loop() {
    Serial.println("🔍 Reading humidity sensors...");

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

    // Serial.println("🌱 Humidity Readings:");
    // Serial.printf(" - Pump 1: %d\n", humidity[0]);
    // Serial.printf(" - Pump 2: %d\n", humidity[1]);
    // Serial.printf(" - Pump 3: %d\n", humidity[2]);
    // Serial.printf(" - Pump 4: %d\n", humidity[3]);

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
    Serial.println("🌱 Humidity Level:");
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
        Serial.printf(" - Pump %d: Raw: %d | Min: %d | Max: %d\n",
            i, humidity[i], pumps[i].min_sensor_reading, pumps[i].max_sensor_reading);
    }

    String jsonData;
    serializeJson(jsonDoc, jsonData);    
    uploadSensorData(jsonData);
    
    // Fully disable Wi-Fi and restore ADC functionality
    disableWiFiAndRestoreADC();
    
    // Delay before next loop
    Serial.println("⏳ Waiting before next cycle...");
    delay(10000);
}
