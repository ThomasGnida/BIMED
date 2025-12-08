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

// SpO2 and HR calculation constants
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// SpO2 calculation buffers
#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;


void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("Initializing modules...");

    // --- LCD ---
    tft.init(240, 240);
    tft.setRotation(2);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);

    // --- Ultrasonic ---
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
}

void measureSPO2() {
    static bool initialized = false;
    static unsigned long lastDisplayUpdate = 0;

    if (!initialized) {
        myWire.begin();
        if (!HRSensor.begin(myWire, I2C_SPEED_STANDARD)) {
            tft.fillScreen(ST77XX_BLACK);
            tft.setCursor(0, 0);
            tft.println("MAX30102 not found!");
            while (1);
        }
        HRSensor.setup(60, 4, 2, 100, 411, 4096);
        HRSensor.setPulseAmplitudeRed(0x0A);
        HRSensor.setPulseAmplitudeGreen(0);
        initialized = true;
    }

    long irValue = HRSensor.getIR();

    if (irValue < 50000) {
        if (millis() - lastDisplayUpdate > 500) {
            tft.fillScreen(ST77XX_BLACK);
            tft.setCursor(10, 100);
            tft.setTextSize(2);
            tft.setTextColor(ST77XX_WHITE);
            tft.println("Place finger");
            lastDisplayUpdate = millis();
            Serial.println("No finger detected");
        }
    }

    for (byte i = 0; i < BUFFER_LENGTH; i++) {

        while (!HRSensor.available()) {
            HRSensor.check();
        }
        redBuffer[i] = HRSensor.getRed();
        //Serial.println(HRSensor.getRed());
        irBuffer[i] = HRSensor.getIR();
        //Serial.println(HRSensor.getIR());
        HRSensor.nextSample();
    }

    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGTH, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    Serial.println("HR:" + String(heartRate));
    Serial.println("SPO2" + String(spo2));

}


void loop() {

    measureSPO2();
    delay(100);
}