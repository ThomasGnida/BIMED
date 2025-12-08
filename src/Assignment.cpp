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
        HRSensor.setup(0x1F, 4, 2, 400, 411, 4096); // Configure sensor with 6.4mA power, 4 sample average, Red+IR mode, 400 samples/sec, 411uS pulse width, 4096nA ADC range.
        HRSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
        HRSensor.setPulseAmplitudeIR(0x1F); // Set IR LED to a higher power
        initialized = true;
    }

    // Continuously read sensor data
    for (byte i = 0; i < BUFFER_LENGTH; i++) {
        while (!HRSensor.available()) {
            HRSensor.check();
        }
        redBuffer[i] = HRSensor.getRed();
        irBuffer[i] = HRSensor.getIR();
        HRSensor.nextSample();
    }

    // Calculate heart rate and SpO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGTH, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Display results
    if (validSPO2 && validHeartRate) {
        if (millis() - lastDisplayUpdate > 1000) { // Update every second
            tft.fillScreen(ST77XX_BLACK);
            tft.setCursor(10, 80);
            tft.setTextSize(3);
            tft.setTextColor(ST77XX_WHITE);
            tft.print("HR: ");
            tft.println(heartRate);
            tft.setCursor(10, 120);
            tft.print("SpO2: ");
            tft.println(spo2);
            lastDisplayUpdate = millis();

            Serial.print("HR: ");
            Serial.print(heartRate);
            Serial.print(", SpO2: ");
            Serial.println(spo2);
        }
    } else {
        if (millis() - lastDisplayUpdate > 500) {
            tft.fillScreen(ST77XX_BLACK);
            tft.setCursor(10, 100);
            tft.setTextSize(2);
            tft.setTextColor(ST77XX_WHITE);
            tft.println("Place finger");
            lastDisplayUpdate = millis();
            Serial.println("No finger or invalid data");
        }
    }

}


void loop() {

    measureSPO2();
    delay(50);
}