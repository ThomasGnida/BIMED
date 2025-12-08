#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#define TFT_CS D3
#define TFT_RST D2
#define TFT_DC D7
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// --- Ultrasonic Pins ---
#define ECHO_PIN D0
#define TRIG_PIN D1
// --- MAX30102 PINS on I2C1 (SDA=D4, SCL=D5) ---
MAX30105 HRSensor;
TwoWire myWire(&i2c1_inst, D4, D5);
static float lastDistance = 0.0;
static float currentDistance = 0.0;
static unsigned long lastMeasurementTime = 0;

// SpO2 and HR calculation constants
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
static uint8_t i = 0;

// SpO2 calculation buffers
#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  uint32_t duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0){
    return -1; // Indicate timeout or no reading
  }
  float distance_cm = duration * 0.0343 / 2; //cm/us divided by 2 for go and return
  return distance_cm;
}

float measureSpeed(float oldDistance, float newDistance, float secondsInterval) {
  if (secondsInterval <= 0) {
    return 0; // Prevent division by zero
  }
  return (newDistance - oldDistance) / secondsInterval; // cm/s positive values = approaching, negative = receding
}


void measureSPO2(uint8_t current_index) {
    redBuffer[current_index] = HRSensor.getRed();
    irBuffer[current_index] = HRSensor.getIR();
    HRSensor.nextSample();
}

void setup() {
    static bool initialized = false;
  // Basic serial + display initialization first so we can show errors
  Serial.begin(115200);
  delay(300);
  tft.init(240, 240);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  Serial.println("Initializing modules...");

  // --- I2C / MAX30102 ---
  if (!initialized) {
    myWire.begin();
    if (!HRSensor.begin(myWire, I2C_SPEED_STANDARD)) {
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, 0);
      tft.println("MAX30102 not found!");
      while (1);
    }
    HRSensor.setup(0x1F, 4, 2, 100, 411, 4096);
    HRSensor.setPulseAmplitudeRed(0x0A);
    HRSensor.setPulseAmplitudeGreen(0);
    initialized = true;
  }

  // --- Ultrasonic ---
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // initialize timing for speed calculation
  lastMeasurementTime = millis();
}

void loop() {
    measureSPO2(i);
    if(i == BUFFER_LENGTH -1) {
        i = 0;
        maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGTH, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
        Serial.println("HR:" + String(heartRate));
        Serial.println("SPO2" + String(spo2));
    } else {
        i++;
    }
    unsigned long currentTime = millis();
    lastDistance = currentDistance;
    currentDistance = measureDistance();
    Serial.println("Distance (cm): " + String(currentDistance) + " | Speed (cm/s): " + String(measureSpeed(lastDistance, currentDistance, (currentTime - lastMeasurementTime) / 1000.0)) );
    lastMeasurementTime = currentTime;
}