/*
  Smart HVAC Control System with MQTT/Node-RED Integration
  ESP32 + DHT22 + MQTT + Node-RED Dashboard

  3-state logic:
  - COOL  : AC ON, Heater OFF
  - HEAT  : Heater ON, AC OFF
  - IDLE  : Both OFF

  Modes:
  - AUTO   : Temperature compared to setpoint (FPGA-like FSM)
  - MANUAL : Commands from MQTT (COOL / HEAT / OFF)
*/

#include <Arduino.h>
#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>

// ---------- WiFi ----------
const char* ssid     = "Wokwi-GUEST";
const char* password = "";

// ---------- MQTT ----------
const char* mqtt_broker    = "test.mosquitto.org";
const int   mqtt_port      = 1883;
const char* mqtt_client_id = "ESP32_SmartHVAC_001";

const char* topic_temperature = "smarthvac/temperature";
const char* topic_humidity    = "smarthvac/humidity";
const char* topic_setpoint    = "smarthvac/setpoint";
const char* topic_mode        = "smarthvac/mode";      // AUTO / MANUAL
const char* topic_action      = "smarthvac/action";    // COOL / HEAT / IDLE
const char* topic_power       = "smarthvac/power";     // ON / OFF
const char* topic_command     = "smarthvac/command";   // COOL / HEAT / OFF / AUTO

// ---------- Pins ----------
#define DHTPIN          4
#define DHTTYPE         DHT22
#define COOL_PIN        2     // Relay/LED for AC (cooling)
#define HEAT_PIN        5     // Relay/LED for Heater
#define STATUS_LED_PIN  15    // System OK indicator

// ---------- System Parameters ----------
float SETPOINT         = 26.0;         // Target temperature (°C)
const float HYSTERESIS = 0.7;          // Hysteresis band (°C)
const unsigned long READ_INTERVAL  = 3000;  // Sensor read
const unsigned long MQTT_INTERVAL  = 2000;  // Publish interval

// ---------- Objects ----------
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient mqtt(espClient);

// ---------- State ----------
unsigned long lastReadTime = 0;
unsigned long lastMqttTime = 0;

float currentTemp = 0.0;
float currentHum  = 0.0;

bool manualOverride = false;      // false=AUTO, true=MANUAL

enum HvacAction {
  ACTION_IDLE = 0,
  ACTION_COOL,
  ACTION_HEAT
};

HvacAction currentAction = ACTION_IDLE;

// ---------- Function Prototypes ----------
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void applyAction(HvacAction action);
void publishAll();
void fpgaLikeLogic(float temperature);

// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  dht.begin();

  pinMode(COOL_PIN, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  digitalWrite(COOL_PIN, LOW);
  digitalWrite(HEAT_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════╗");
  Serial.println("║   Smart HVAC (AC + Heater) with MQTT / Node-RED  ║");
  Serial.println("║   ESP32 + DHT22 + 3-State FPGA Logic Simulation  ║");
  Serial.println("╚══════════════════════════════════════════════════╝");
  Serial.println();

  setupWiFi();

  mqtt.setServer(mqtt_broker, mqtt_port);
  mqtt.setCallback(mqttCallback);

  Serial.println("\n--- System Ready ---\n");
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop() {
  unsigned long now = millis();

  // MQTT connection
  if (!mqtt.connected()) {
    reconnectMQTT();
  }
  mqtt.loop();

  // Read sensor
  if (now - lastReadTime >= READ_INTERVAL) {
    lastReadTime = now;

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (isnan(t) || isnan(h)) {
      Serial.println("⚠ ERROR: Sensor read failed!");
      return;
    }

    currentTemp = t;
    currentHum  = h;

    // Automatic control (FPGA-like logic)
    if (!manualOverride) {
      fpgaLikeLogic(currentTemp);
    }

    // ---- Terminal Output ----
    Serial.println("┌───────────────────────────────────────────────┐");
    Serial.print  ("│ Temperature: "); Serial.print(currentTemp, 1); Serial.println(" °C                 │");
    Serial.print  ("│ Humidity   : "); Serial.print(currentHum, 1);  Serial.println(" %                  │");
    Serial.print  ("│ Setpoint   : "); Serial.print(SETPOINT, 1);    Serial.println(" °C                 │");
    Serial.print  ("│ Mode       : "); Serial.print(manualOverride ? "MANUAL" : "AUTO  ");
    Serial.println("                        │");

    Serial.print("│ Action     : ");
    if (currentAction == ACTION_COOL)      Serial.print("COOLING");
    else if (currentAction == ACTION_HEAT) Serial.print("HEATING");
    else                                   Serial.print("IDLE   ");
    Serial.println("                        │");

    Serial.print("│ Power      : ");
    if (currentAction == ACTION_IDLE)      Serial.print("OFF");
    else                                   Serial.print("ON ");
    Serial.println("                           │");
    Serial.println("└───────────────────────────────────────────────┘\n");
  }

  // Publish MQTT
  if (now - lastMqttTime >= MQTT_INTERVAL) {
    lastMqttTime = now;
    publishAll();
  }
}

// =====================================================
// WiFi + MQTT
// =====================================================

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
    Serial.println("\n✗ WiFi Connection Failed (continuing offline)!");
  }
}

void reconnectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Connecting to MQTT broker... ");

    if (mqtt.connect(mqtt_client_id)) {
      Serial.println("connected!");

      mqtt.subscribe(topic_command);
      mqtt.subscribe(topic_setpoint);

      mqtt.publish(topic_mode, manualOverride ? "MANUAL" : "AUTO");
      publishAll();   // Send initial data
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 5 seconds...");
      delay(5000);
    }
  }
}

// =====================================================
// MQTT Callback
// =====================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg = "";

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("📩 MQTT [");
  Serial.print(t);
  Serial.print("] = ");
  Serial.println(msg);

  if (t == topic_command) {
    msg.trim();
    msg.toUpperCase();

    if (msg == "COOL") {
      manualOverride = true;
      applyAction(ACTION_COOL);
      mqtt.publish(topic_mode, "MANUAL");
      Serial.println(">>> MANUAL: COOL requested");

    } else if (msg == "HEAT") {
      manualOverride = true;
      applyAction(ACTION_HEAT);
      mqtt.publish(topic_mode, "MANUAL");
      Serial.println(">>> MANUAL: HEAT requested");

    } else if (msg == "OFF") {
      manualOverride = true;
      applyAction(ACTION_IDLE);
      mqtt.publish(topic_mode, "MANUAL");
      Serial.println(">>> MANUAL: OFF requested");

    } else if (msg == "AUTO") {
      manualOverride = false;
      mqtt.publish(topic_mode, "AUTO");
      Serial.println(">>> AUTO mode resumed");
      fpgaLikeLogic(currentTemp);
    }
  }

  if (t == topic_setpoint) {
    float newSp = msg.toFloat();
    if (newSp > 10 && newSp < 40) {
      SETPOINT = newSp;
      Serial.print(">>> Setpoint updated to: ");
      Serial.print(SETPOINT, 1);
      Serial.println(" °C");
      if (!manualOverride) {
        fpgaLikeLogic(currentTemp);
      }
    } else {
      Serial.println("✗ Ignored invalid setpoint");
    }
  }
}

// =====================================================
// FPGA-like 3-state HVAC Logic (AUTO mode)
// =====================================================

void fpgaLikeLogic(float temperature) {
  HvacAction next = ACTION_IDLE;

  // Temperature above setpoint -> COOL
  if (temperature > SETPOINT + HYSTERESIS) {
    next = ACTION_COOL;
  }
  // Temperature below setpoint -> HEAT
  else if (temperature < SETPOINT - HYSTERESIS) {
    next = ACTION_HEAT;
  }
  // Otherwise, temperature within comfort band -> IDLE
  else {
    next = ACTION_IDLE;
  }

  if (next != currentAction) {
    applyAction(next);
    Serial.print("🔧 AUTO: T="); Serial.print(temperature);
    Serial.print(" SP="); Serial.print(SETPOINT);
    Serial.print(" → ");
    if (next == ACTION_COOL)  Serial.println("COOL");
    if (next == ACTION_HEAT)  Serial.println("HEAT");
    if (next == ACTION_IDLE)  Serial.println("IDLE");
  }
}


// =====================================================
// Relay / LED + MQTT Status
// =====================================================

void applyAction(HvacAction action) {
  currentAction = action;

  bool coolOn = (action == ACTION_COOL);
  bool heatOn = (action == ACTION_HEAT);

  digitalWrite(COOL_PIN, coolOn ? HIGH : LOW);
  digitalWrite(HEAT_PIN, heatOn ? HIGH : LOW);

  // Action topic
  if (action == ACTION_COOL)      mqtt.publish(topic_action, "COOL",  true);
  else if (action == ACTION_HEAT) mqtt.publish(topic_action, "HEAT",  true);
  else                            mqtt.publish(topic_action, "IDLE",  true);

  // Power topic (ON if any relay is active)
  if (coolOn || heatOn) mqtt.publish(topic_power, "ON",  true);
  else                  mqtt.publish(topic_power, "OFF", true);
}

// Publish all sensor & status data
void publishAll() {
  if (!mqtt.connected()) return;

  char buf[16];

  dtostrf(currentTemp, 5, 1, buf);
  mqtt.publish(topic_temperature, buf);

  dtostrf(currentHum, 5, 1, buf);
  mqtt.publish(topic_humidity, buf);

  dtostrf(SETPOINT, 5, 1, buf);
  mqtt.publish(topic_setpoint, buf, true);

  mqtt.publish(topic_mode, manualOverride ? "MANUAL" : "AUTO", true);

  if (currentAction == ACTION_COOL)      mqtt.publish(topic_action, "COOL", true);
  else if (currentAction == ACTION_HEAT) mqtt.publish(topic_action, "HEAT", true);
  else                                   mqtt.publish(topic_action, "IDLE", true);

  if (currentAction == ACTION_IDLE) mqtt.publish(topic_power, "OFF", true);
  else                              mqtt.publish(topic_power, "ON",  true);

  Serial.println("📤 MQTT: All data published");
}
