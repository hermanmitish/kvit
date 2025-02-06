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
    config.frame_size = FRAMESIZE_QVGA;  // Start with QVGA for stability
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
    Serial.println("🚀 ESP32 Moisture Monitoring System Starting...");
    setupCamera(); // Initialize camera
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

void captureAndUpload(String jsonData) {
    Serial.println("📸 Capturing image...");
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("❌ Camera capture failed!");
        return;
    }

    Serial.println("📤 Sending image and sensor data...");
    
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "multipart/form-data; boundary=ESP32BOUNDARY");

    // Create multipart form data
    String boundary = "ESP32BOUNDARY";
    String formData = "--" + boundary + "\r\n";
    formData += "Content-Disposition: form-data; name=\"json\"\r\n\r\n";
    formData += jsonData + "\r\n";

    formData += "--" + boundary + "\r\n";
    formData += "Content-Disposition: form-data; name=\"image\"; filename=\"snapshot.jpg\"\r\n";
    formData += "Content-Type: image/jpeg\r\n\r\n";

    // Send multipart data
    WiFiClient client;
    http.begin(client, serverUrl);
    int contentLength = formData.length() + fb->len + boundary.length() + 6;
    
    http.addHeader("Content-Length", String(contentLength));
    int httpResponseCode = http.POST(formData);
    client.write(fb->buf, fb->len);
    client.print("\r\n--" + boundary + "--\r\n");

    esp_camera_fb_return(fb);
    
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
        sensorReading["moisture"] = moisture[i];
    }

    String jsonData;
    serializeJson(jsonDoc, jsonData);

    Serial.println("📤 Sending data to server...");
    
    captureAndUpload(jsonData);
    
    // Fully disable Wi-Fi and restore ADC functionality
    disableWiFiAndRestoreADC();
    
    // Delay before next loop
    Serial.println("⏳ Waiting before next cycle...");
    delay(10000);
}