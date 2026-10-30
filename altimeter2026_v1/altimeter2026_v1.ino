// altimeter2026_v1.ino

#include <Arduino.h>
#include <FastLED.h>
#include <Adafruit_ICM20649.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "FS.h"   // SD file system lib
#include "SD.h"   // SD card lib
#include "SPI.h"  // spi lib for connecting to SD card
#include <Servo.h>
#include <TFT_eSPI.h>  // Graphics and font library for ILI9341 driver chip -- has been modified for waveshare lcd 
#include <SPI.h>
#include <Encoder.h>
#include "RunningAverage.h"  // running average library: calculates average and max / min easily

int versionNumber = 02;  //firmware version

// needed for SD card to work, pins card is wired to SPI1
int sck = 10;   //
int miso = 12;  //
int mosi = 11;  //
int cs = 15;    //
bool SDinit;

Servo servo1;        // brakes
#define SERVO_PIN 3
int servo1Position = 1500;  //start middle

Servo servo2;        // dual deploy latch
#define SERVO2_PIN 2
int servo2Position = 2000;  //start open

#define SWITCH_PIN 8    //encoder switch
bool switchClick = 0;  // store switch state

Encoder myEnc(6, 7);  //rotary encoder

Adafruit_ICM20649 icm;
uint16_t measurement_delay_us = 65535;  // Delay between measurements for testing

Adafruit_BMP3XX bmp;

// leds
#define NUM_LEDS 1
#define DATA_PIN 22  // rgb led

// Define the array of leds
CRGB leds[NUM_LEDS];

#define TFT_GREY 0x5AEB  // New colour
#define PIXELPERLINE 24  //on lcd for font 4

TFT_eSPI tft = TFT_eSPI();  // Invoke library

long oldPosition = -999;  // for encoder

//====================
float RELEASE_ALT = 0;  // trigger height in m (depricated)
int FLIGHT_NUM;         // number of flights or boots of the arduino

#define SERIAL_DEBUG 1  // print serial monitor data  toggle

#define M2FEET 3.28084  // meters to feet conversion factor

#define LAUNCH_THRESHOLD 10.0    // Launch detection threshold in meters/second
#define SAMPLE_WINDOW 20         // Number of samples to check for rate calculation
#define DRIFT_THRESHOLD 0.00635  // 0.25 inches in meters - threshold for resetting base_alt

#define APOGEE_THRESHOLD -20.0  // 20 m/s ? guess

// servo position variables
int RELEASE_POSITION = 1000;   // open
int LATCHED_POSITION 2000;   // close
int DOORS_SHUT = 1925;
int DOORS_OPEN = 1220;

int SEALEVELPRESSURE_HPA = 1021;  // https://weather.us/observations/new-mexico/pressure-qnh/20230920-1700z.html#obs-detail-723650
//#define SEALEVELPRESSURE_HPA 1019 //typical ABQ value

char numString[4];    // short string for loggin fn
char rocketLog[20];   // numbered rocket log file path name
char dataString[75];  // holds SD logging string
char buffer[10000];

int i = 0;                  //ram buffer length indicator
RunningAverage altRA(500);  // store 500 measurments (25 seconds of data at 20hz sample rate) in a runningAverage buffer

float ini_alt = 0;        // initial ground level
float dif_alt = 0;        // altitude difference from initial in m
float base_alt = 0;       // percieved ground level altitude in m
float triggered_alt = 0;  // alt trigger occurred in m
float current_alt = 0;
float deltaT;

#define loopms 15                      // how many milliseconds per measurement loop 20ms = 50hz 10ms = 100hZ 1ms5 = 66hz
#define serialms 200                    // milliseconds per serial print loop
#define menums 250                     // milliseconds permenu loop
unsigned long lastloop = millis();     // loop timer
unsigned long triggeredTime = 0;       // trigger timer
unsigned long serialLoop = millis();    // longer serial output loop
unsigned long menuLoop = millis();     // menu update loop
unsigned long apogeeTime = millis();  // time when apogee happened
unsigned long currentTime;

bool launch_detected = false;  // New flag for launch detection
bool apogee_detected = false;  // apogee has occured
float rate = 0.0;

float chuteDelay = 10.0;
int menuState = 0;
int prevMenuState = 0;
int knobInc = 0;
bool lcdUpdate = 1;
//====================

//++++++++++++
//logging function to append data to the end of the file
void appendFile(SDClass &fs, const char *path, const char *message) {
  //Serial.printf("Appending to file: %s\n", path);  // don't do serial upon sucess, slows it down
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.print("Failed to open");
    Serial.print(path);
    Serial.println(" for appending");
    return;
  }
  file.seek(EOF);
  if (file.print(message)) {
    //Serial.print("");   // don't do serial for speedup
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

//logging function to read 4 characters from a file on card
int read3File(SDClass &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return -1;
  }
  char fileContent[10];
  int i = 0;
  while (file.available() && i < 4) {  //reads first 4 characters of a textfile, contents need to be 4 ascii digits
    fileContent[i] = file.read();
    i++;
  }
  file.close();
  return atoi(fileContent);  // return file digits as integer
}

int writeFile(SDClass &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return (0);
  }
  if (file.println(message)) {
    //Serial.print("");   // don't do serial for speedup
    file.close();
    return (1);
  } else {
    Serial.println("write failed");
    file.close();
    return (0);
  }
}
//++++++++++++


void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Altimeter startup");

  pinMode(SWITCH_PIN, INPUT_PULLUP);  // set encoder button pin high so presses are detected by pulling down to gnd

  tft.init();
  tft.setRotation(2);  // usbport at bottom
  tft.fillScreen(TFT_GREY);

  // Set "cursor" at top left corner of display (0,0) and select font 4
  // (cursor will move to next line automatically during printing with 'tft.println'
  //  or stay on the line is there is room for the text with tft.print)
  tft.setCursor(0, 0, 4);
  // Set the font colour to be white with a black background, set text size multiplier to 1
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(1);

  tft.print("Altimeter V");
  tft.println(versionNumber);

  Serial.println("SD setup starting");
  SPI1.setRX(miso);
  SPI1.setTX(mosi);
  SPI1.setSCK(sck);
  SDinit = SD.begin(cs, SPI1);

  if (!SDinit) {
    Serial.println("Card Mount Failed");
    tft.println("SD Fail");
    return;
  }
  Serial.println("Card OK");
  tft.println("SD ok");

  //=============
  // read flight variables from SD card
  SEALEVELPRESSURE_HPA = read3File(SD, "/HPA.txt");  // ground level air pressure in text file
  Serial.print("Read HPA.txt and got SEALEVELPRESSURE_HPA = ");
  Serial.println(SEALEVELPRESSURE_HPA);
  tft.print("HPA:");
  tft.println(SEALEVELPRESSURE_HPA);

  RELEASE_ALT = read3File(SD, "/eject.txt");  // desired release height in ft in text file
  Serial.print("Read eject.txt and got release  = ");
  Serial.println(RELEASE_ALT);
  RELEASE_ALT = RELEASE_ALT / M2FEET;  // convert feet to meters
  tft.print("eject:");
  tft.println(RELEASE_ALT);

  FLIGHT_NUM = read3File(SD, "/fn.txt");  // desired release height in ft in text file
  Serial.print("Read fn.txt and got FN  = ");
  Serial.println(FLIGHT_NUM);

  FLIGHT_NUM = FLIGHT_NUM + 1;
  //FLIGHT_NUM = 1;
  tft.print("FN:");
  tft.println(FLIGHT_NUM);

  if (SD.exists("/fn.txt")) {
    SD.remove("/fn.txt");
  }

  sprintf(numString, "%04d", FLIGHT_NUM);
  writeFile(SD, "/fn.txt", numString);
  sprintf(rocketLog, "FN%04drckt.txt", FLIGHT_NUM);
  //=============

  servo1.attach(SERVO_PIN, 1000, 2000);
  servo2.attach(SERVO2_PIN, 1000, 2000);

  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20649 chip");
    tft.println("ACCEL fail");

    while (1) {
      delay(10);
    }
    Serial.println("ICM20649 Found!");
    tft.println("ACCEL ok");
    icm.setAccelDLPF();
  }

  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    tft.println("BMP fail");
    while (1)
      ;
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("BMP390 Found!");
  tft.println("BMP ok");

  FastLED.addLeds<WS2812, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  leds[0] = CRGB::Red;
  FastLED.show();

  //=============
  Serial.println("averaging sensor altitude readings ...");
  tft.println("init BMP");
  for (int i = 0; i < 300; i++) {  // take readings for 15 sec
    bmp.performReading();
    altRA.addValue(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    delay(50);
  }

  ini_alt = altRA.getAverageLast(100);  // set base altitude average of last 5 sec. of readings
  base_alt = ini_alt;                   // store ground level
  altRA.clear();                        //clear running average data, max and min will only be from this point on


  // bmp.performReading();
  // base_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  Serial.println(base_alt);
  tft.print("GND: ");
  tft.println(base_alt * M2FEET);

  delay(5000);

  // log flight initial data to card
  //  appendFile(SD, "/rocket.txt", "FlightNumber, Groundlevel Alt(ft), Release Alt(ft), SLHPA\n");
  appendFile(SD, rocketLog, "FlightNumber, Groundlevel Alt(ft), Release Alt(ft), SLHPA\n\n");

  sprintf(dataString, "%d, %4.2f, %4.2f, %d\n", FLIGHT_NUM, base_alt * M2FEET, RELEASE_ALT * M2FEET, SEALEVELPRESSURE_HPA);
  appendFile(SD, rocketLog, dataString);  // headers for data file
  appendFile(SD, rocketLog, "FlightNum, seconds, altitude(ft), ground level (ft), max(ft), launch, release,  X, Y, Z(m/s^2)\n");

  //=============
}

void loop() {
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  leds[0] = CRGB::Green;

  FastLED.show();

  servo1.write(servo1Position);
  servo2.write(servo2Position);


  if (millis() - lastloop > loopms) {
    currentTime = millis();
    deltaT = (currentTime - lastloop) / 1000.0;  // Convert to seconds
    lastloop = currentTime;

    bmp.performReading();
    icm.getEvent(&accel, &gyro, &temp);

    current_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    altRA.addValue(current_alt);  // Store actual altitude

    // Calculate  rate of change
    if (altRA.getCount() >= SAMPLE_WINDOW) {
      float newest = altRA.getAverageLast(SAMPLE_WINDOW / 2);  // Average of newest half
      float older = altRA.getAverageLast(SAMPLE_WINDOW);       // Average of all SAMPLE_WINDOW readings

      // Calculate velocity (m/s) using current deltaT
      rate = (newest - older) / deltaT;
    }

    if (!launch_detected) {  //hasn't launched, keep updating Ground level
      // Check if we need to correct for drift
      dif_alt = current_alt - base_alt;
      if (abs(dif_alt) > DRIFT_THRESHOLD) {
        base_alt = altRA.getAverage();  // Reset base using running average
        dif_alt = current_alt - base_alt;
      }

      if (rate > LAUNCH_THRESHOLD) {
        launch_detected = true;

        if (SERIAL_DEBUG) {
          Serial.println("Launch detected!");
          Serial.print("Launch velocity: ");
          Serial.print(rate);
          Serial.println(" m/s");
        }
      }
    }
  } else {
    dif_alt = current_alt - base_alt;
  }

  if (launch_detected && !apogee_detected) {  //
    if(altRA.getMax() - altRA.getAverageLast(6) > 50) {
      // if (rate < APOGEE_THRESHOLD) {    // old rate method
      apogee_detected = true;
    apogeeTime = millis();
  }
}

if (apogee_detected) {
  if (millis() - apogeeTime > chuteDelay) {
    servo2Position = RELEASE_POSITION;
  }
}
//LOGGING
// fill datastring with logging data //FlightNum, seconds, altitude(ft), ground level (ft), max(ft), launch,  X, Y, Z(m/s^2)
sprintf(dataString, "FN%d, %4.3f, %4.2f, %4.2f, %4.2f, %d,%4.1f,%4.1f,%4.1f\n ",
        FLIGHT_NUM, currentTime * .001, dif_alt * M2FEET, base_alt * M2FEET, (altRA.getMax() - ini_alt) * M2FEET, launch_detected,
        (float)accel.acceleration.x, (float)accel.acceleration.y, (float)accel.acceleration.z);


if (i <= 9000) {
  i += sprintf(buffer + i, dataString);
}

if (i > 9000) {
  appendFile(SD, rocketLog, buffer);
  i = 0;
}

if (millis() - menuLoop > menums) {
  menuLoop = millis();

  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    if (oldPosition < newPosition) knobInc = 1;
    if (oldPosition > newPosition) knobInc = -1;
    oldPosition = newPosition;
  } else {
    knobInc = 0;
  }
  switchClick = !digitalRead(SWITCH_PIN);  // read switch presses 1=press, 0 = no press

  ///MENU
  if (lcdUpdate) {  // refresh base display
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(5, 0, 4);
    tft.setTextColor(TFT_BLUE, TFT_BLACK);  //blue is actual red
    //tft.print("Altimeter V");
    //tft.println(versionNumber);
    tft.print("Flight ");
    tft.println(FLIGHT_NUM);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.setCursor(0, 1 * PIXELPERLINE, 4);
    tft.print("Alt: ");
    tft.println(dif_alt * M2FEET);

    tft.setCursor(0, 2 * PIXELPERLINE, 4);
    tft.print("Max: ");
    tft.print((altRA.getMax() - ini_alt) * M2FEET);

    tft.setCursor(0, 3 * PIXELPERLINE, 4);
    tft.print("HPA: ");
    tft.println(SEALEVELPRESSURE_HPA);

    tft.setCursor(0, 4 * PIXELPERLINE, 4);
    tft.print("Chute: ");
    tft.println(chuteDelay);

    tft.setCursor(0, 5 * PIXELPERLINE, 4);
    tft.print("Servo1: ");
    tft.println(servo1Position);

    tft.setCursor(0, 6 * PIXELPERLINE, 4);
    tft.print("Servo2: ");
    tft.println(servo2Position);

    tft.setCursor(0, 7 * PIXELPERLINE, 4);
    tft.print("Brakes: ");
    if (servo1Position == DOORS_OPEN) {
      tft.print("OPEN");
    } else if (servo1Position == DOORS_SHUT) {
      tft.print("SHUT");
    } else {
      tft.print(map(servo1Position, 2000, 1000, 0, 100));
      tft.print("%");
    }

    tft.setCursor(0, 8 * PIXELPERLINE, 4);
    tft.print("Latch: ");
    if (servo2Position == RELEASE_POSITION) {
      tft.print("OPEN ");
    } else if (servo2Position == LATCHED_POSITION) {
      tft.print("CLOSE");
    } else {
      tft.print(map(servo2Position, 1220, 1925, 0, 100));
      tft.print("%");
    }

    tft.setCursor(0, 9 * PIXELPERLINE, 4);
    tft.print("   SET   ");
  }

  switch (menuState) {
    case 0:                             // no hilite
      if (switchClick) menuState = 20;  // jump to highlite w/ click
      //lcdUpdate = false;
      break;

    case 10:  // wraparound to end
      menuState = 80;
      break;

    case 20:                                   // HPA line 3
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.setCursor(0, 3 * PIXELPERLINE, 4);
      tft.print("HPA: ");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      tft.println(SEALEVELPRESSURE_HPA);
      if (switchClick) menuState = menuState + 5;
      menuState = menuState + 10 * knobInc;
      break;
    case 25:  //edit HPA
      tft.setCursor(0, 3 * PIXELPERLINE, 4);
      tft.print("HPA: ");
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.println(SEALEVELPRESSURE_HPA);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      if (switchClick) menuState = menuState - 5;
      SEALEVELPRESSURE_HPA = SEALEVELPRESSURE_HPA + knobInc;
      break;

    case 30:  //chute line 4
      tft.setCursor(0, 4 * PIXELPERLINE, 4);
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.print("Chute: ");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      tft.println(chuteDelay);
      if (switchClick) menuState = menuState + 5;
      menuState = menuState + 10 * knobInc;
      break;
    case 35:  //edit chute
      tft.setCursor(0, 4 * PIXELPERLINE, 4);
      tft.print("Chute: ");
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.println(chuteDelay);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      if (switchClick) menuState = menuState - 5;
      chuteDelay = chuteDelay + 0.1 * knobInc;
      break;

    case 40:  //servo1 line 5
      tft.setCursor(0, 5 * PIXELPERLINE, 4);
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.print("Servo1: ");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      tft.println(servo1Position);
      if (switchClick) menuState = menuState + 5;
      menuState = menuState + 10 * knobInc;
      break;
    case 45:  //edit servo1
      tft.setCursor(0, 5 * PIXELPERLINE, 4);
      tft.print("Servo1: ");
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.println(servo1Position);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      if (switchClick) menuState = menuState - 5;
      servo1Position = servo1Position + 5 * knobInc;
      break;

    case 50:  //servo2 line 6
      tft.setCursor(0, 6 * PIXELPERLINE, 4);
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.print("Servo2: ");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      tft.println(servo2Position);
      if (switchClick) menuState = menuState + 5;
      menuState = menuState + 10 * knobInc;
      break;
    case 55:  //edit servo2
      tft.setCursor(0, 6 * PIXELPERLINE, 4);
      tft.print("Servo2: ");
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.println(servo2Position);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      if (switchClick) menuState = menuState - 5;
      servo2Position = servo2Position + 5 * knobInc;
      break;

    case 60:
      tft.setCursor(0, 7 * PIXELPERLINE, 4);
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.print("Brakes: ");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      if (servo1Position == DOORS_OPEN) {
        tft.print("OPEN");
      } else if (servo1Position == DOORS_SHUT) {
        tft.print("SHUT");
      } else {
        tft.print(map(servo1Position, 1220, 1925, 0, 100));
        tft.print("%");
      }

      if (switchClick) menuState = menuState + 5;
      menuState = menuState + 10 * knobInc;
      break;

    case 65:
      tft.setCursor(0, 7 * PIXELPERLINE, 4);
      tft.print("Brakes: ");
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      if (servo1Position == DOORS_OPEN) {
        tft.print("OPEN ");
      } else if (servo1Position == DOORS_SHUT) {
        tft.print("SHUT");
      } else {
        tft.print(map(servo1Position, 1220, 1925, 0, 100));
        tft.print("%");
      }
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain

      if (knobInc) {
        switch (knobInc) {
          case 1:
            servo1Position = DOORS_OPEN;
            break;
          case -1:
            servo1Position = DOORS_SHUT;
            break;
        }
      }
      if (switchClick) menuState = menuState - 5;

      break;


    case 70:
      tft.setCursor(0, 8 * PIXELPERLINE, 4);
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.print("Latch: ");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
      if (servo2Position == RELEASE_POSITION) {
        tft.print("OPEN");
      } else if (servo2Position == LATCHED_POSITION) {
        tft.print("CLOSE");
      } else {
        tft.print(map(servo2Position, 2000, 1000, 0, 100));
        tft.print("%");
      }

      if (switchClick) menuState = menuState + 5;
      menuState = menuState + 10 * knobInc;
      break;

    case 75:
      tft.setCursor(0, 8 * PIXELPERLINE, 4);
      tft.print("Latch: ");
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      if (servo2Position == RELEASE_POSITION) {
        tft.print("OPEN ");
      } else if (servo2Position == LATCHED_POSITION) {
        tft.print("CLOSE");
      } else {
        tft.print(map(servo2Position, 2000, 1000, 0, 100));
        tft.print("%");
      }
      tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain

      if (knobInc) {
        switch (knobInc) {
          case 1:
            servo2Position = LATCHED_POSITION;
            break;
          case -1:
            servo2Position = RELEASE_POSITION;
            break;
        }
      }
      if (switchClick) menuState = menuState - 5;

      break;

    case 80:
      tft.setCursor(0, 9 * PIXELPERLINE, 4);
      tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
      tft.print("   SET   ");
      if (switchClick) menuState = 0;
      menuState = menuState + 10 * knobInc;
      break;

    case 90:
      menuState = 20;  //back to top
      break;

    default:  // state 0
      // if nothing else matches, do the default
      // default is optional
      break;
  }

  lcdUpdate = true;
  if (prevMenuState == menuState) lcdUpdate = false;
  prevMenuState = menuState;
}

if (millis() - serialLoop > serialms) {
  serialLoop = millis();
  if (SERIAL_DEBUG) {

    Serial.print(" hZ:");
    float deltaMs = (float)(millis() - lastloop);
    if (deltaMs > 0) {
      Serial.print(1000.0 / deltaMs);
    } else {
      Serial.print("--");
    }
    Serial.print(" Launched: ");
    Serial.print(launch_detected);
    Serial.print(" Rate: ");
    if (altRA.getCount() >= SAMPLE_WINDOW) {
      float rate = (altRA.getAverageLast(SAMPLE_WINDOW / 2) - altRA.getAverageLast(SAMPLE_WINDOW)) / deltaT;
      Serial.print(rate);
    }
    Serial.print(" Base: ");
    Serial.print(base_alt * M2FEET);
    Serial.print("ft Drift: ");
    Serial.print(dif_alt * M2FEET);
    Serial.print("ft ");
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");
    Serial.print(dataString);
  }
}
}
