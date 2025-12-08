/*#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

/*
    If everything works as it should, the display should display which sensors are operational or not.
    The same information should be shown in the serial monitor as well @115200 baud.
#1#
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



void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Initializing modules...");

  // --- LCD Test ---
  tft.init(240, 240);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 20);
  tft.println("Screen operational");
  delay(1000);

  // --- Ultrasonic Test ---
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  if (pulseIn(ECHO_PIN, HIGH, 3000000) > 0) {
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 100);
    tft.println("DistanceSensor operational");
    Serial.println("DistanceSensor operational");
  } else {
    tft.setCursor(40, 100);
    tft.setTextColor(ST77XX_RED);
    tft.println("DistanceSensor FAIL OR TIMEOUT!");
    Serial.println("DistanceSensor FAIL OR TIMEOUT!");
  }

  // --- MAX30102 Test ---
  myWire.begin();
  if (!HRSensor.begin(myWire, I2C_SPEED_STANDARD)) {
    tft.setCursor(40, 180);
    tft.setTextColor(ST77XX_RED);
    tft.println("MAX30102 not found. Check wiring!");
    Serial.println("MAX30102 not found. Check wiring!");
    while (1)
      ;
  } else {
    tft.setCursor(40, 180);
    tft.setTextColor(ST77XX_WHITE);
    tft.println("MAX30102 operational");
    Serial.println("MAX30102 operational");
  }
}

void loop() {

  delay(1);
}*/