#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

/*
    If everything works as it should, the display should display which sensors are operational or not.
    The same information should be shown in the serial monitor as well @115200 baud.
*/
// If wired identical to the image in the moodle course, these pin definitions are correct.
// Otherwise, adapt as neccessary.
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
static float lastDistance = 0;
static float currentDistance = 0;
static float lastMeasurementTime = 0;



void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Initializing modules...");
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  uint32_t duration = pulseIn(ECHO_PIN, HIGH, 1000000);
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


void loop() {
  unsigned long currentTime = millis();
  lastDistance = currentDistance;
  currentDistance = measureDistance();
  Serial.println("Distance (cm): " + String(currentDistance) + " | Speed (cm/s): " + String(measureSpeed(lastDistance, currentDistance, (currentTime - lastMeasurementTime) / 1000.0)) );
  lastMeasurementTime = currentTime;

  delay(60);
}