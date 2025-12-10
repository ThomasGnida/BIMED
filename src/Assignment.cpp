#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <math.h>

#define TFT_CS D3
#define TFT_RST D2
#define TFT_DC D7
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#define ECHO_PIN D0
#define TRIG_PIN D1

// --- MAX30102 PINS on I2C1 (SDA=D4, SCL=D5) ---
MAX30105 HRSensor;
TwoWire myWire(&i2c1_inst, D4, D5);

// Distance tracking
float lastDistance = 0.0;
float currentDistance = 0.0;
static unsigned long lastDistanceRead = 0;
unsigned long lastMeasurementTime = 0;
float total_distance = 0.0;
float currentSpeed = 0.0;


// SpO2 calculation buffers
#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t spo2 = 0;
int8_t validSPO2 = 0;
int32_t heartRate = 0;
int8_t validHeartRate = 0;
bool fingerDetected = false;

uint8_t bufferIndex = 0;
bool bufferReady = false;
uint32_t lastHeartRateCalc = 0;
uint16_t HR_CALC_INTERVAL = 500;
uint32_t lastDisplayUpdate = 0;


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
  return (newDistance - oldDistance) / secondsInterval; // cm/s positive values = approaching, negative = receding
}


bool collectSensorSample() {
  uint32_t redValue = HRSensor.getRed();
  uint32_t irValue = HRSensor.getIR();
  HRSensor.nextSample();
  
  if (redValue > 15000 && irValue > 15000) {
    redBuffer[bufferIndex] = redValue;
    irBuffer[bufferIndex] = irValue;
    
    if(!bufferReady & bufferIndex +1 == BUFFER_LENGTH){
      bufferReady = true; //Arrays are filled for the first time
    }
    bufferIndex = (bufferIndex + 1) % BUFFER_LENGTH;
    return true;
  }
  return false;
}

void updateDisplay() {
    lastDisplayUpdate = millis();

    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);

    // Display HR and SpO2
    if (!fingerDetected){
        tft.setCursor(10, 20);
        tft.println("No finger detected");
    } else if (validSPO2 && validHeartRate) {
        tft.setCursor(10, 20);
        tft.print("  HR: ");
        tft.print(heartRate);
        tft.println(" bpm");

        tft.setCursor(10, 50);
        tft.print("SpO2: ");
        tft.print(spo2);
        tft.println(" %");

    } else {
        tft.setCursor(10, 20);
        tft.println("Invalid HR/Spo2 values");
    }

    // Display Distance
    tft.setCursor(10, 100);
    if (currentDistance >= 0) {
        tft.print("Dist: ");
        tft.print(currentDistance, 1);
        tft.println(" cm");

    } else {
        tft.println("Dist: --");
    }

    // Display Speed
    tft.setCursor(10, 130);
    tft.print("Speed: ");
    tft.print(currentSpeed, 1);
    tft.println(" cm/s");

    tft.setCursor(10, 160);
    tft.print("Total Dist: ");
    tft.print(total_distance, 2); // convert to meters
    tft.println(" cm");
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
    HRSensor.setup(0x2F, 4, 2, 50, 411, 4096);
    HRSensor.setPulseAmplitudeRed(0x1A);
    HRSensor.setPulseAmplitudeGreen(0);
    initialized = true;
  }

  // --- Ultrasonic ---
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  lastMeasurementTime = millis();
}

void loop() {
  unsigned long currentTime = millis();  
  fingerDetected = collectSensorSample();
  
  // Calculate HR and SpO2 every 500ms when buffer is full and ready
  if (bufferReady && (currentTime - lastHeartRateCalc >= HR_CALC_INTERVAL)) {
    lastHeartRateCalc = currentTime;
    
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, 
      BUFFER_LENGTH, 
      redBuffer, 
      &spo2, 
      &validSPO2, 
      &heartRate, 
      &validHeartRate
    );
  }
  
  if (currentTime - lastDistanceRead >= 100) { // Every 100ms
    lastDistanceRead = currentTime;
    
    float newDistance = measureDistance();
    
    if (newDistance >= 0.0) {
      float timeDifference = (currentTime - lastMeasurementTime) / 1000.0;
      
      if (timeDifference > 0) {
        currentSpeed = measureSpeed(lastDistance, newDistance, timeDifference);
        float distanceDelta = abs(newDistance - lastDistance);
        
        if (distanceDelta > 0.5 && distanceDelta < 100) { // Ignore noise and unrealistic jumps
          total_distance += distanceDelta;
        }
      }
     
      currentDistance = newDistance;
      lastDistance = newDistance;
      lastMeasurementTime = currentTime;
    }
  }

  if (currentTime - lastDisplayUpdate >= 500) { 
    lastDisplayUpdate = currentTime;
    updateDisplay();
  }
}