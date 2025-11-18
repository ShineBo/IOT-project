/*
  Smart Fan Control System with MQTT/Node-RED Integration
  ESP32 + DHT22 + MQTT + Node-RED Dashboard
  
  Features:
  - DHT22 temperature & humidity monitoring
  - MQTT communication with Node-RED
  - Automatic fan control based on temperature threshold
  - Manual override via MQTT commands
  - Real-time data publishing to dashboard
  - Simulates FPGA logic in software for testing
*/

#include <Arduino.h>
#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials (for simulation)
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Broker settings
const char* mqtt_broker = "test.mosquitto.org";  // Public broker for testing
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_SmartFan_001";

// MQTT Topics
const char* topic_temperature = "smartfan/temperature";
const char* topic_humidity = "smartfan/humidity";
const char* topic_fan_status = "smartfan/fan/status";
const char* topic_fan_command = "smartfan/fan/command";
const char* topic_mode = "smartfan/mode";
const char* topic_threshold = "smartfan/threshold";

// Pin Definitions
#define DHTPIN 4              // DHT data pin
#define DHTTYPE DHT22         // DHT sensor type
#define FAN_PIN 2             // Fan control (relay/LED)
#define STATUS_LED_PIN 15     // System status indicator

// System Parameters
float TEMP_THRESHOLD = 30.0;           // Temperature threshold (°C)
const float HYSTERESIS = 1.0;          // Prevent rapid switching
const unsigned long READ_INTERVAL = 3000;   // Sensor read interval (ms)
const unsigned long MQTT_INTERVAL = 2000;   // MQTT publish interval (ms)

// System Objects
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient mqtt(espClient);

// System State
unsigned long lastReadTime = 0;
unsigned long lastMqttTime = 0;
bool fanState = false;
bool manualOverride = false;
float currentTemp = 0.0;
float currentHum = 0.0;

// Function prototypes
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void controlFan(bool state);
void publishSensorData();
void fpgaLogic(float temperature);

void setup() {
  Serial.begin(115200);
  
  // Initialize DHT sensor
  dht.begin();
  
  // Configure pins
  pinMode(FAN_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Initial states
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Startup message
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  Smart Fan Control with MQTT/Node-RED ║");
  Serial.println("║  ESP32 + DHT22 + FPGA Logic Simulation║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
  
  // Connect to WiFi
  setupWiFi();
  
  // Setup MQTT
  mqtt.setServer(mqtt_broker, mqtt_port);
  mqtt.setCallback(mqttCallback);
  
  Serial.println("\n--- System Ready ---\n");
  digitalWrite(STATUS_LED_PIN, HIGH);
  
  delay(2000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Maintain MQTT connection
  if (!mqtt.connected()) {
    reconnectMQTT();
  }
  mqtt.loop();
  
  // Read sensor at intervals
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    
    // Read sensor
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    // Validate readings
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("⚠ ERROR: Sensor read failed!");
      return;
    }
    
    currentTemp = temperature;
    currentHum = humidity;
    
    // FPGA Logic Simulation (automatic mode only)
    if (!manualOverride) {
      fpgaLogic(temperature);
    }
    
    // Print to Serial
    Serial.println("┌─────────────────────────────────────────┐");
    Serial.print("│ Temperature: ");
    Serial.print(temperature, 1);
    Serial.println(" °C                  │");
    Serial.print("│ Humidity:    ");
    Serial.print(humidity, 1);
    Serial.println(" %                    │");
    Serial.print("│ Threshold:   ");
    Serial.print(TEMP_THRESHOLD, 1);
    Serial.println(" °C                  │");
    Serial.print("│ Fan Status:  ");
    Serial.print(fanState ? "ON " : "OFF");
    Serial.print(manualOverride ? " (MANUAL)" : " (AUTO)  ");
    Serial.println("      │");
    Serial.println("└─────────────────────────────────────────┘\n");
  }
  
  // Publish to MQTT at intervals
  if (currentTime - lastMqttTime >= MQTT_INTERVAL) {
    lastMqttTime = currentTime;
    publishSensorData();
  }
}

void setupWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ WiFi Connection Failed!");
  }
}

void reconnectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Connecting to MQTT broker...");
    
    if (mqtt.connect(mqtt_client_id)) {
      Serial.println(" Connected!");
      
      // Subscribe to command topics
      mqtt.subscribe(topic_fan_command);
      mqtt.subscribe(topic_threshold);
      
      Serial.println("✓ Subscribed to command topics");
      
      // Publish initial status
      mqtt.publish(topic_mode, manualOverride ? "MANUAL" : "AUTO");
      
    } else {
      Serial.print(" Failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("📩 MQTT Message [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);
  
  // Handle fan command
  if (String(topic) == topic_fan_command) {
    if (message == "ON") {
      manualOverride = true;
      controlFan(true);
      mqtt.publish(topic_mode, "MANUAL");
      Serial.println(">>> Manual Override: Fan ON");
    } 
    else if (message == "OFF") {
      manualOverride = true;
      controlFan(false);
      mqtt.publish(topic_mode, "MANUAL");
      Serial.println(">>> Manual Override: Fan OFF");
    }
    else if (message == "AUTO") {
      manualOverride = false;
      mqtt.publish(topic_mode, "AUTO");
      Serial.println(">>> Automatic Mode Resumed");
      // Re-evaluate temperature immediately
      fpgaLogic(currentTemp);
    }
  }
  
  // Handle threshold change
  if (String(topic) == topic_threshold) {
    float newThreshold = message.toFloat();
    if (newThreshold > 0 && newThreshold < 50) {
      TEMP_THRESHOLD = newThreshold;
      Serial.print(">>> Threshold updated to: ");
      Serial.print(TEMP_THRESHOLD);
      Serial.println(" °C");
      
      // Re-evaluate if in auto mode
      if (!manualOverride) {
        fpgaLogic(currentTemp);
      }
    }
  }
}

void fpgaLogic(float temperature) {
  // Simulates FPGA digital logic for fan control
  // Uses hysteresis to prevent rapid switching
  
  bool newFanState = fanState;
  
  if (temperature > TEMP_THRESHOLD + HYSTERESIS) {
    newFanState = true;
  } else if (temperature < TEMP_THRESHOLD - HYSTERESIS) {
    newFanState = false;
  }
  // If within hysteresis band, maintain current state
  
  if (newFanState != fanState) {
    controlFan(newFanState);
    Serial.print("🔧 FPGA Logic: Temperature ");
    Serial.print(temperature, 1);
    Serial.print("°C → Fan ");
    Serial.println(newFanState ? "ON" : "OFF");
  }
}

void controlFan(bool state) {
  fanState = state;
  digitalWrite(FAN_PIN, state ? HIGH : LOW);
  
  // Publish fan status to MQTT
  mqtt.publish(topic_fan_status, state ? "ON" : "OFF", true);
}

void publishSensorData() {
  if (mqtt.connected()) {
    // Publish temperature
    char tempStr[8];
    dtostrf(currentTemp, 5, 1, tempStr);
    mqtt.publish(topic_temperature, tempStr);
    
    // Publish humidity
    char humStr[8];
    dtostrf(currentHum, 5, 1, humStr);
    mqtt.publish(topic_humidity, humStr);
    
    // Publish current mode
    mqtt.publish(topic_mode, manualOverride ? "MANUAL" : "AUTO");
    
    Serial.println("📤 Data published to MQTT");
  }
}