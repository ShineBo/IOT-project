/*
  Enhanced Smart Fan Control System with LCD & RGB LED
  ESP32 + DHT22 + LCD I2C + RGB LED + Visual Feedback
  
  Features:
  - DHT22 temperature & humidity monitoring
  - 16x2 LCD I2C display for real-time data
  - RGB LED color-coded temperature indication
  - LED indicators for system states
  - Serial plotter-compatible output
  - Temperature trend tracking
  - Manual override capability
*/

#include <Arduino.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>

// Pin Definitions
#define DHTPIN 4              // DHT data pin
#define DHTTYPE DHT22         // DHT sensor type
#define FAN_PIN 2             // Fan control (red LED)
#define STATUS_LED_PIN 15     // System status (green LED)
#define ALERT_LED_PIN 16      // Temperature alert (yellow LED)
#define BUTTON_PIN 5          // Manual override button

// RGB LED pins
#define RGB_RED_PIN 25
#define RGB_GREEN_PIN 26
#define RGB_BLUE_PIN 27

// System Parameters
const float TEMP_THRESHOLD = 30.0;      // Temperature threshold (°C)
const float HYSTERESIS = 1.0;           // Prevent rapid switching
const unsigned long READ_INTERVAL = 2000;  // Sensor read interval (ms)
const unsigned long BLINK_INTERVAL = 500;  // Status LED blink rate
const unsigned long LCD_UPDATE_INTERVAL = 1000; // LCD update rate

// Temperature ranges for RGB LED
const float TEMP_COOL = 25.0;      // Below this = Blue (cool)
const float TEMP_COMFORT = 30.0;   // Between cool and hot = Green (comfortable)
// Above TEMP_COMFORT = Red (hot)

// System Objects
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 columns, 2 rows

// System State
unsigned long lastReadTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastLCDUpdate = 0;
bool statusLedState = false;
bool fanState = false;
bool manualOverride = false;
bool lastButtonState = HIGH;
float tempHistory[5] = {0};  // Store last 5 readings
int historyIndex = 0;

// Custom LCD characters
byte degreeSymbol[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b00110,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte fanIcon[8] = {
  0b00100,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00100,
  0b00000
};

byte dropIcon[8] = {
  0b00100,
  0b00100,
  0b01010,
  0b01010,
  0b10001,
  0b10001,
  0b01110,
  0b00000
};

// Function prototypes
void updateLEDs();
void updateRGB(float temp);
void updateLCD(float temp, float hum);
void updateTemperatureHistory(float temp);
float getAverageTemp();
void printSerialData(float temp, float hum);
void checkManualOverride();
void setRGBColor(int red, int green, int blue);

void setup() {
  Serial.begin(115200);
  
  // Initialize DHT sensor
  dht.begin();
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, degreeSymbol);
  lcd.createChar(1, fanIcon);
  lcd.createChar(2, dropIcon);
  
  // Display startup screen
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Fan Ctrl");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  // Configure pins
  pinMode(FAN_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ALERT_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  
  // Initial states
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, HIGH);
  digitalWrite(ALERT_LED_PIN, LOW);
  setRGBColor(0, 0, 255);  // Blue on startup
  
  // Startup message
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   Smart Fan Control System v3.0       ║");
  Serial.println("║   ESP32 + DHT22 + LCD + RGB LED       ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
  Serial.print("Temperature Threshold: ");
  Serial.print(TEMP_THRESHOLD);
  Serial.println(" °C");
  Serial.print("Hysteresis Band: ±");
  Serial.print(HYSTERESIS);
  Serial.println(" °C");
  Serial.println("\nRGB Temperature Ranges:");
  Serial.print("  Blue (Cool):    < ");
  Serial.print(TEMP_COOL);
  Serial.println(" °C");
  Serial.print("  Green (Comfort): ");
  Serial.print(TEMP_COOL);
  Serial.print("-");
  Serial.print(TEMP_COMFORT);
  Serial.println(" °C");
  Serial.print("  Red (Hot):      > ");
  Serial.print(TEMP_COMFORT);
  Serial.println(" °C");
  Serial.println("\n--- System Ready ---\n");
  
  delay(2000);  // Stabilization delay
  lcd.clear();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Status LED heartbeat
  if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = currentTime;
    statusLedState = !statusLedState;
    digitalWrite(STATUS_LED_PIN, statusLedState);
  }
  
  // Check manual override button
  checkManualOverride();
  
  // Read sensor at intervals
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    
    // Read sensor
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    // Validate readings
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("⚠ ERROR: Sensor read failed!");
      digitalWrite(ALERT_LED_PIN, HIGH);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SENSOR ERROR!");
      lcd.setCursor(0, 1);
      lcd.print("Check DHT22");
      setRGBColor(255, 0, 255);  // Magenta for error
      return;
    }
    
    digitalWrite(ALERT_LED_PIN, LOW);
    
    // Update temperature history
    updateTemperatureHistory(temperature);
    float avgTemp = getAverageTemp();
    
    // Fan control logic (if not in manual override)
    if (!manualOverride) {
      // Hysteresis logic to prevent rapid switching
      if (temperature > TEMP_THRESHOLD + HYSTERESIS) {
        fanState = true;
      } else if (temperature < TEMP_THRESHOLD - HYSTERESIS) {
        fanState = false;
      }
      // If within hysteresis band, maintain current state
    }
    
    // Apply fan state
    digitalWrite(FAN_PIN, fanState ? HIGH : LOW);
    
    // Update visual indicators
    updateLEDs();
    updateRGB(temperature);
    
    // Update LCD
    if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
      lastLCDUpdate = currentTime;
      updateLCD(temperature, humidity);
    }
    
    // Print formatted data
    printSerialData(temperature, humidity);
  }
}

void setRGBColor(int red, int green, int blue) {
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  analogWrite(RGB_BLUE_PIN, blue);
}

void updateRGB(float temp) {
  if (temp < TEMP_COOL) {
    // Cool temperature - Blue
    setRGBColor(0, 0, 255);
  } else if (temp < TEMP_COMFORT) {
    // Comfortable temperature - Green
    // Smoothly transition from Blue to Green
    float ratio = (temp - TEMP_COOL) / (TEMP_COMFORT - TEMP_COOL);
    int blue = (int)(255 * (1 - ratio));
    int green = (int)(255 * ratio);
    setRGBColor(0, green, blue);
  } else {
    // Hot temperature - Red
    // Smoothly transition from Green to Red
    float ratio = min((temp - TEMP_COMFORT) / 5.0, 1.0);  // 5 degree range for transition
    int red = (int)(255 * ratio);
    int green = (int)(255 * (1 - ratio));
    setRGBColor(red, green, 0);
  }
}

void updateLCD(float temp, float hum) {
  lcd.clear();
  
  // First line: Temperature with custom degree symbol and fan icon
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temp, 1);
  lcd.write(0);  // Degree symbol
  lcd.print("C ");
  
  if (fanState) {
    lcd.write(1);  // Fan icon
    lcd.print("ON");
  } else {
    lcd.print("   OFF");
  }
  
  // Second line: Humidity with drop icon and mode
  lcd.setCursor(0, 1);
  lcd.write(2);  // Drop icon
  lcd.print(":");
  lcd.print(hum, 1);
  lcd.print("% ");
  
  if (manualOverride) {
    lcd.print("MAN");
  } else {
    lcd.print("AUTO");
  }
}

void checkManualOverride() {
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  // Detect button press (falling edge)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    delay(50);  // Debounce
    manualOverride = !manualOverride;
    
    if (manualOverride) {
      fanState = !fanState;  // Toggle fan
      Serial.println("\n>>> MANUAL OVERRIDE ACTIVATED <<<");
      Serial.print("Fan manually set to: ");
      Serial.println(fanState ? "ON" : "OFF");
      
      // Show manual mode on LCD briefly
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MANUAL MODE");
      lcd.setCursor(0, 1);
      lcd.print("Fan: ");
      lcd.print(fanState ? "ON" : "OFF");
      delay(1500);
    } else {
      Serial.println("\n>>> AUTOMATIC MODE RESUMED <<<");
      
      // Show auto mode on LCD briefly
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("AUTO MODE");
      lcd.setCursor(0, 1);
      lcd.print("Resuming...");
      delay(1500);
    }
  }
  
  lastButtonState = currentButtonState;
}

void updateTemperatureHistory(float temp) {
  tempHistory[historyIndex] = temp;
  historyIndex = (historyIndex + 1) % 5;
}

float getAverageTemp() {
  float sum = 0;
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (tempHistory[i] > 0) {
      sum += tempHistory[i];
      count++;
    }
  }
  return (count > 0) ? (sum / count) : 0;
}

void updateLEDs() {
  // Alert LED indicates high temperature
  if (fanState) {
    digitalWrite(ALERT_LED_PIN, HIGH);
  } else {
    digitalWrite(ALERT_LED_PIN, LOW);
  }
}

void printSerialData(float temp, float hum) {
  // Determine temperature trend
  char trend = '→';
  if (historyIndex >= 2) {
    int prevIdx = (historyIndex - 1 + 5) % 5;
    if (temp > tempHistory[prevIdx] + 0.5) trend = '↑';
    else if (temp < tempHistory[prevIdx] - 0.5) trend = '↓';
  }
  
  // Calculate distance from threshold
  float delta = temp - TEMP_THRESHOLD;
  
  // Determine RGB color zone
  String colorZone;
  if (temp < TEMP_COOL) {
    colorZone = "BLUE (Cool)";
  } else if (temp < TEMP_COMFORT) {
    colorZone = "GREEN (Comfort)";
  } else {
    colorZone = "RED (Hot)";
  }
  
  Serial.println("┌─────────────────────────────────────────┐");
  Serial.print("│ Temperature: ");
  Serial.print(temp, 1);
  Serial.print(" °C  ");
  Serial.print(trend);
  Serial.print("  (Δ");
  Serial.print(delta >= 0 ? "+" : "");
  Serial.print(delta, 1);
  Serial.println(" °C) │");
  
  Serial.print("│ Humidity:    ");
  Serial.print(hum, 1);
  Serial.println(" %                    │");
  
  Serial.print("│ Avg Temp:    ");
  Serial.print(getAverageTemp(), 1);
  Serial.println(" °C (5-reading avg)   │");
  
  Serial.print("│ RGB Zone:    ");
  Serial.print(colorZone);
  for (int i = colorZone.length(); i < 23; i++) Serial.print(" ");
  Serial.println("│");
  
  Serial.print("│ Fan Status:  ");
  Serial.print(fanState ? "🔴 ON " : "⚪ OFF");
  if (manualOverride) {
    Serial.println(" (MANUAL)            │");
  } else {
    Serial.println(" (AUTO)              │");
  }
  
  Serial.print("│ System Mode: ");
  Serial.println(manualOverride ? "MANUAL OVERRIDE          │" : "AUTOMATIC CONTROL        │");
  
  Serial.println("└─────────────────────────────────────────┘");
  Serial.println();
}