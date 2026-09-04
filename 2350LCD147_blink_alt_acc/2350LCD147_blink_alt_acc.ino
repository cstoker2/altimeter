/// @file    Blink.ino
/// @brief   Blink the first LED of an LED strip
/// @example Blink.ino

#include <Arduino.h>
#include <FastLED.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "FS.h"   // SD file system lib
#include "SD.h"   // SD card lib
#include "SPI.h"  // spi lib for connecting to SD card
#include <Servo.h>
//#include "src/libraries/TFT_eSPI/TFT_eSPI.h"  // Graphics and font library for ILI9341 driver chip
#include <TFT_eSPI.h>  // Graphics and font library for ILI9341 driver chip
#include <SPI.h>

// needed for SD card to work, pins card is wired to SPI1
int sck = 10;   //
int miso = 12;  //
int mosi = 11;  //
int cs = 15;    //
bool SDinit;

Servo servo1;
#define SERVO_PIN 3

Adafruit_ICM20649 icm;
uint16_t measurement_delay_us = 65535;  // Delay between measurements for testing

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

// How many leds in your strip?
#define NUM_LEDS 1

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI
#define DATA_PIN 22
//#define CLOCK_PIN 13

// Define the array of leds
CRGB leds[NUM_LEDS];




#define TFT_GREY 0x5AEB  // New colour

TFT_eSPI tft = TFT_eSPI();  // Invoke library


void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("BLINK setup starting");

  Serial.println("SD setup starting");
  SPI1.setRX(miso);
  SPI1.setTX(mosi);
  SPI1.setSCK(sck);
  SDinit = SD.begin(cs, SPI1);

  if (!SDinit) {
    Serial.println("Card Mount Failed");
    return;
  }
  Serial.println("Card OK");

servo1.attach(SERVO_PIN, 1000, 2000); 

  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20649 chip");
    while (1) {
      delay(10);
    }
    Serial.println("ICM20649 Found!");
    icm.setAccelDLPF();
  }

  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  tft.init();
  tft.setRotation(1);

  tft.fillScreen(TFT_GREY);

  // Set "cursor" at top left corner of display (0,0) and select font 2
  // (cursor will move to next line automatically during printing with 'tft.println'
  //  or stay on the line is there is room for the text with tft.print)
  tft.setCursor(90, 66, 4);
  // Set the font colour to be white with a black background, set text size multiplier to 1
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  // We can now plot text on screen using the "print" class
  tft.println("Hello World!");

  delay(1000);

  FastLED.addLeds<WS2812, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
}

void loop() {
  Serial.print("BLINK");

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp);

  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  Serial.printf(" Altitude %f, X %f, Y %f, Z %f \n", bmp.readAltitude(SEALEVELPRESSURE_HPA), accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
  // Turn the LED on, then pause
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(500);

servo1.write(1500);

  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
}
