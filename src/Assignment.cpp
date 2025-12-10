#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <math.h>

Adafruit_ST7789 screen = Adafruit_ST7789(D3, D7, D2);
#define ECHO_PIN D0
#define TRIG_PIN D1
MAX30105 HRSensor;
TwoWire myWire(&i2c1_inst, D4, D5);

// Distance tracking
float lastDistance = 0.0;
float currentDistance = 0.0;
uint16_t lastDistanceTime = 0;
float totalDistance = 0.0;
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
uint32_t lastHeartRateCalcTime = 0;
uint16_t HR_CALC_INTERVAL = 500;
uint32_t lastDisplayUpdate = 0;



float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  uint32_t duration = pulseIn(ECHO_PIN, HIGH, 30000);

  return duration * 0.0343 / 2; //cm/us divided by 2 for go and return
}

bool collectSensorSample() {
  uint32_t redValue = HRSensor.getRed();
  uint32_t irValue = HRSensor.getIR();
  HRSensor.nextSample();
  
  if (redValue > 15000 && irValue > 15000) {
    redBuffer[bufferIndex] = redValue;
    irBuffer[bufferIndex] = irValue;
    if(!bufferReady & bufferIndex +1 == BUFFER_LENGTH){
      bufferReady = true; 
    }
    bufferIndex = (bufferIndex + 1) % BUFFER_LENGTH;
    return true;
  }
  return false;
}

void updateDisplay() {
  screen.fillScreen(ST77XX_BLACK);
    if (!fingerDetected){
        screen.setCursor(10, 20);
        screen.println("No finger detected");
    } else if (validSPO2 && validHeartRate) {
        screen.setCursor(10, 20);
        screen.print("  HR: ");
        screen.print(heartRate);
        screen.println(" bpm");

        screen.setCursor(10, 50);
        screen.print("SpO2: ");
        screen.print(spo2);
        screen.println(" %");
    } else {
        screen.setCursor(10, 20);
        screen.println("Invalid Spo2 values");
    }

    screen.setCursor(10, 100);
    if (currentDistance >= 0) {
        screen.print("Dist: ");
        screen.print(currentDistance, 1);
        screen.println(" cm");
    } else {
        screen.println("Dist: --");
    }
    
    screen.setCursor(10, 130);
    screen.print("Speed: ");
    screen.print(currentSpeed, 1);
    screen.println(" cm/s");
    screen.setCursor(10, 160);
    screen.print("Total Dist: ");
    screen.print(totalDistance, 1);
    screen.println(" cm");
}

void setup() {
  bool initialized = false;
  Serial.begin(115200);
  delay(300);
  screen.init(240, 240);
  screen.setRotation(2);
  screen.fillScreen(ST77XX_BLACK);
  screen.setTextColor(ST77XX_WHITE);
  screen.setTextSize(2);
  // --- I2C / MAX30102 ---
  if (!initialized) {
    myWire.begin();
    if (!HRSensor.begin(myWire, I2C_SPEED_STANDARD)) {
      screen.setCursor(0, 0);
      screen.println("MAX30102 not found!");
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

  lastDistanceTime = millis();
}

void loop() {
  unsigned long currentTime = millis();  
  fingerDetected = collectSensorSample();
  
  // Calculate HR and SpO2 every 500ms when buffer is full and ready
  if (bufferReady && (currentTime - lastHeartRateCalcTime >= HR_CALC_INTERVAL)) {
    lastHeartRateCalcTime = currentTime;
    
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
  
  if (currentTime - lastDistanceTime >= 100) { // Every 100ms
    
    float newDistance = measureDistance();
    
    if (newDistance >= 0.0) {
      float timeDifference = (currentTime - lastDistanceTime) / 1000.0;
      
      if (timeDifference > 0) {
        currentSpeed = (newDistance - lastDistance) / timeDifference;

        float distanceDelta = abs(newDistance - lastDistance);
        
        if (distanceDelta > 0.5 && distanceDelta < 100) { // Ignore noise and unrealistic jumps
          totalDistance += distanceDelta;
        }
      }
     
      currentDistance = newDistance;
      lastDistance = newDistance;
      lastDistanceTime = currentTime;
    }
  }

  if (currentTime - lastDisplayUpdate >= 500) { 
    lastDisplayUpdate = currentTime;
    updateDisplay();
  }
}