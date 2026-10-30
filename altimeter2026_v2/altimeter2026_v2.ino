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
#include <Battery.h>

int versionNumber = 02;  //firmware version

// needed for SD card to work, pins card is wired to SPI1
int sck = 10;   //
int miso = 12;  //
int mosi = 11;  //
int cs = 15;    //
bool SDinit;

#define M2FEET 3.28084  // meters to feet conversion factor

#define BATT_SENSE_PIN 26  // (10.41 + 6.55) / 6.55 = 2.589

Servo brakeServo;  // brakes
#define BRAKE_SERVO_PIN 3
int brakeServoPosition = 1500;  //start middle

Servo latchServo;  // dual deploy latch
#define LATCH_SERVO_PIN 2
int latchServoPosition = 2000;  //start open

#define SWITCH_PIN 8   //encoder switch
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
int TARGET_ALT = 750;  // trigger height in m
int FLIGHT_NUM;        // number of flights or boots of the arduino

#define SERIAL_DEBUG 1  // print serial monitor data  toggle

#define LAUNCH_THRESHOLD 10.0    // Launch detection threshold in meters/second
#define SAMPLE_WINDOW 20         // Number of samples to check for rate calculation
#define DRIFT_THRESHOLD 0.00635  // 0.25 inches in meters - threshold for resetting gnd_alt
#define APOGEE_THRESHOLD -20.0   // 20 m/s ? guess

// servo position variables
int LATCH_OPEN = 1000;   // open
int LATCH_CLOSE = 2000;  // shut
int BRAKE_OPEN = 1220;   // full brake
int BRAKE_CLOSE = 1925;  // brakes off

int SEALEVELPRESSURE_HPA = 1017;  // https://weather.us/observations/new-mexico/pressure-qnh/20230920-1700z.html#obs-detail-723650
//#define SEALEVELPRESSURE_HPA 1019 //typical ABQ value 1017 gave good agreement with competition altimeter.

char numString[5];    // short string for loggin fn
char battString[8];   // short battery "7.55v" string
char rocketLog[20];   // numbered rocket log file path name
char dataString[75];  // holds SD logging string
char buffer[10000];

int i = 0;                  //ram buffer length indicator
RunningAverage altRA(500);  // store 500 measurments (25 seconds of data at 20hz sample rate) in a runningAverage buffer

float ini_alt = 0;        // initial averaged alt
float dif_alt = 0;        // altitude difference from initial in m
float gnd_alt = 0;        // percieved ground level altitude in m
float triggered_alt = 0;  // alt trigger occurred in m
float current_alt = 0;    // live reading of alt
float deltaT = 0.1;       // time step, avoid divide by zero

#define loopms 15                         // how many milliseconds per measurement loop 20ms = 50hz 10ms = 100hZ 1ms5 = 66hz
#define serialms 200                      // milliseconds per serial print loop
#define menums 250                        // milliseconds permenu loop
unsigned long lastloop = millis();        // loop timer
unsigned long triggeredTime = 0;          // trigger timer
unsigned long serialLoop = millis();      // longer serial output loop
unsigned long menuLoop = millis();        // menu update loop
unsigned long apogeeTime = millis();      // time when apogee happened
unsigned long dispUpdateTime = millis();  // time menu display updated
unsigned long currentTime;

bool launch_detected = false;  // New flag for launch detection
bool apogee_detected = false;  // apogee has occured
float rate = 0.0;

int chuteDelay = 1000;
int menuState = 0;
int prevMenuState = 0;
int knobInc = 0;
bool lcdUpdate = 1;

//pinMode(BATT_SENSE_PIN, INPUT);
Battery battery(6000, 8400, BATT_SENSE_PIN, 12);  //2s lipo on 12bit adc


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

//logging function to read 5 characters from a file on card
int read3File(SDClass &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return -1;
  }
  char fileContent[10];
  int i = 0;
  while (file.available() && i < 5) {  //reads first 5 characters of a textfile, contents need to be 5 ascii digits
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

void serialLogTransfer() {
  char fileList[50][20];
  int fileCount = 0;

  //Scan for log files
  auto scanFiles = [&]() {
    fileCount = 0;
    File root = SD.open("/");
    if (!root) {
      Serial.println("Failed to open root directory");
      return;
    }
    while (fileCount < 50) {
      File entry = root.openNextFile();
      if (!entry) break;  // no more files
      if (entry.isDirectory()) {
        entry.close();
        continue;
      }
      const char *name = entry.name();
      int len = strlen(name);
      // match files ending in "log.txt" (case-sensitive, matches FN#####log.txt)
      if (len >= 7 && strcmp(name + len - 7, "log.txt") == 0) {
        strncpy(fileList[fileCount], name, 19);
        fileList[fileCount][19] = '\0';
        fileCount++;
      }
      entry.close();
    }
    root.close();
  };

  // Print menu
  auto printMenu = [&]() {
    Serial.println();
    Serial.println("=== Log File Menu ===");
    Serial.print("Files found: ");
    Serial.println(fileCount);
    for (int i = 0; i < fileCount; i++) {
      Serial.print(" ");
      Serial.print(i + 1);
      Serial.print(") ");
      Serial.println(fileList[i]);
    }
    Serial.println("Enter 1-50 to send, R to refresh, X to exit:");
  };

  // --- Send a file ---
  auto sendFile = [&](int index) {
    char path[22];
    snprintf(path, sizeof(path), "/%s", fileList[index]);

    File file = SD.open(path, FILE_READ);
    if (!file) {
      Serial.print("Failed to open: ");
      Serial.println(path);
      return;
    }

    long fileSize = file.size();
    Serial.println();
    Serial.print("--- BEGIN ");
    Serial.print(fileList[index]);
    Serial.print(" (");
    Serial.print(fileSize);
    Serial.println(" bytes) ---");

    uint8_t buf[512];
    long bytesSent = 0;
    while (file.available()) {
      int bytesRead = file.read(buf, sizeof(buf));
      if (bytesRead > 0) {
        Serial.write(buf, bytesRead);
        bytesSent += bytesRead;
      }
    }
    file.close();

    Serial.println();
    Serial.print("--- END ");
    Serial.print(fileList[index]);
    Serial.print(" --- ");
    Serial.print(bytesSent);
    Serial.println(" bytes sent");
  };

  // --- Main menu loop ---
  scanFiles();
  printMenu();

  while (true) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input.equalsIgnoreCase("X")) {
        Serial.println("Exiting log menu.");
        break;

      } else if (input.equalsIgnoreCase("R")) {
        scanFiles();
        printMenu();

      } else {
        int selection = input.toInt();
        if (selection >= 1 && selection <= fileCount) {
          sendFile(selection - 1);
          printMenu();  // re-show menu after transfer
        } else {
          Serial.print("Invalid selection: ");
          Serial.println(input);
          Serial.println("Enter 1-50 to send, R to refresh, X to exit:");
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  if (SERIAL_DEBUG) Serial.println("Altimeter startup");

  pinMode(SWITCH_PIN, INPUT_PULLUP);  // set encoder button pin high so presses are detected by pulling down to gnd

  tft.init();
  tft.setRotation(2);  // usbport at bottom
  tft.fillScreen(TFT_GREY);
  // Set "cursor" at top left corner of display (0,0) and select font 4
  tft.setCursor(0, 0, 4);
  // Set the font colour to be white with a black background, set text size multiplier to 1
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(1);

  tft.print("Altimeter V");
  tft.println(versionNumber);

  pinMode(BATT_SENSE_PIN, INPUT);
  analogReadResolution(12);
  battery.begin(3300, (10.41 + 6.55) / 6.55, &asigmoidal);

  tft.println("Battery:");
  tft.print(battery.voltage() / 1000.0);
  tft.print("v ");
  // tft.print(battery.level());
  // tft.println("%");

  if (SERIAL_DEBUG) Serial.println("SD setup starting");
  SPI1.setRX(miso);
  SPI1.setTX(mosi);
  SPI1.setSCK(sck);
  SDinit = SD.begin(cs, SPI1);

  if (!SDinit) {
    if (SERIAL_DEBUG) Serial.println("Card Mount Failed");
    tft.println("SD Fail");
    return;
  }
  if (SERIAL_DEBUG) Serial.println("Card OK");
  tft.println("SD ok");

  //=============
  // read flight variables from SD card
  if (SD.exists("/hpa.txt")) {
    SEALEVELPRESSURE_HPA = read3File(SD, "/hpa.txt");
    SD.remove("/hpa.txt");
  }  // ground level air pressure in text file
  sprintf(numString, "%05d", SEALEVELPRESSURE_HPA);
  writeFile(SD, "/hpa.txt", numString);
  tft.print("HPA:");
  tft.println(SEALEVELPRESSURE_HPA);

  if (SD.exists("/target.txt")) {
    TARGET_ALT = read3File(SD, "/target.txt");
    SD.remove("/target.txt");
  }  // desired release height in ft in text file
  sprintf(numString, "%05d", TARGET_ALT);
  writeFile(SD, "/target.txt", numString);
  tft.print("target:");
  tft.println(TARGET_ALT);

  if (SD.exists("/deploydelay.txt")) {
    chuteDelay = read3File(SD, "/deploydelay.txt");
    SD.remove("/deploydelay.txt");
  }  // desired release height in ft in text file
  sprintf(numString, "%05d", chuteDelay);
  writeFile(SD, "/deploydelay.txt", numString);
  tft.print("deploy: ");
  tft.println(chuteDelay);

  FLIGHT_NUM = read3File(SD, "/fn.txt");  // desired release height in ft in text file

  FLIGHT_NUM = FLIGHT_NUM + 1;
  //FLIGHT_NUM = 1;    //for resetting
  tft.print("FN:");
  tft.println(FLIGHT_NUM);

  if (SD.exists("/fn.txt")) {
    SD.remove("/fn.txt");
  }
  sprintf(numString, "%05d", FLIGHT_NUM);
  writeFile(SD, "/fn.txt", numString);

  sprintf(rocketLog, "FN%05dlog.txt", FLIGHT_NUM);  // setup logfile name

  if (SD.exists("/latchopen.txt")) { LATCH_OPEN = read3File(SD, "/latchopen.txt"); }
  if (SD.exists("/latchclose.txt")) { LATCH_CLOSE = read3File(SD, "/latchcshut.txt"); }
  if (SD.exists("/brakeopen.txt")) { BRAKE_OPEN = read3File(SD, "/brakeopen.txt"); }
  if (SD.exists("/brakeshut.txt")) { BRAKE_CLOSE = read3File(SD, "/brakeshut.txt"); }

  if (SERIAL_DEBUG) {
    Serial.print("SEALEVELPRESSURE_HPA = ");
    Serial.println(SEALEVELPRESSURE_HPA);
    Serial.print("Read deploydelay.txt and got delay  = ");
    Serial.println(chuteDelay);
    Serial.print("Read target.txt and got target  = ");
    Serial.println(TARGET_ALT);
    Serial.print("Read fn.txt and got FN  = ");
    Serial.println(FLIGHT_NUM);
  }

  brakeServo.attach(BRAKE_SERVO_PIN, 1000, 2000);
  latchServo.attach(LATCH_SERVO_PIN, 1000, 2000);

  if (!icm.begin_I2C()) {
    if (SERIAL_DEBUG) Serial.println("Failed to find ICM20649 chip");
    tft.println("ACCEL fail");

    while (1) {
      delay(10);
    }
    if (SERIAL_DEBUG) Serial.println("ICM20649 Found!");
    tft.println("ACCEL ok");
    icm.setAccelDLPF();
  }

  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    if (SERIAL_DEBUG) Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    tft.println("BMP fail");
    while (1)
      ;
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (SERIAL_DEBUG) Serial.println("BMP390 Found!");
  tft.println("BMP ok");

  FastLED.addLeds<WS2812, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  leds[0] = CRGB::Red;
  FastLED.show();

  //=============
  if (SERIAL_DEBUG) Serial.println("averaging sensor altitude readings ...");
  tft.println("averaging...");
  for (int i = 0; i < 300; i++) {  // take readings for 15 sec
    bmp.performReading();
    altRA.addValue(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    delay(50);
  }

  ini_alt = altRA.getAverageLast(100);  // set base altitude average of last 5 sec. of readings
  gnd_alt = ini_alt;
  altRA.clear();  //clear running average data, max and min will only be from this point on

  if (SERIAL_DEBUG) Serial.println(gnd_alt);
  tft.print("GND: ");
  tft.println(gnd_alt * M2FEET);

  tft.print("Xfer..? ");

  Serial.println("Press T within 10 seconds to enter log transfer menu...");
  unsigned long promptStart = millis();
  while (millis() - promptStart < 10000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'T' || c == 't') {
        serialLogTransfer();
        break;
      }
    }
  }

  tft.fillScreen(TFT_BLACK);

  // log data header to card

  sprintf(dataString, "FN: %d, GND: %4.2f, Target: %4.2f, HPA: %d, Delay: %d\n", FLIGHT_NUM, gnd_alt * M2FEET, TARGET_ALT * M2FEET, SEALEVELPRESSURE_HPA, chuteDelay);
  appendFile(SD, rocketLog, dataString);  // headers for data file
  appendFile(SD, rocketLog, "FN, seconds, altitude(ft), rate(m/s), ground level (ft), max(ft), brake%, launch, apogee, latchsrvo, X, Y, Z(m/s^2)\n");
}

void loop() {
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  leds[0] = CRGB::Green;
  FastLED.show();

  brakeServo.write(brakeServoPosition);
  latchServo.write(latchServoPosition);

  if (millis() - lastloop > loopms) {  // Main loop every 15ms
    currentTime = millis();
    deltaT = (currentTime - lastloop) / 1000.0;  // Convert to seconds
    lastloop = currentTime;

    bmp.performReading();
    icm.getEvent(&accel, &gyro, &temp);

    current_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    altRA.addValue(current_alt);  // Store actual altitude

    // Calculate  rate of change
    if (altRA.getCount() >= SAMPLE_WINDOW) {                   // calculate once 20samples stored, should happen within 0.3sec
      float newest = altRA.getAverageLast(SAMPLE_WINDOW / 2);  // Average of newest half
      float older = altRA.getAverageLast(SAMPLE_WINDOW);       // Average of all SAMPLE_WINDOW readings

      rate = (newest - older) / deltaT;  // height change in m/sec
    }

    // test for flight states:
    // no launch
    if (!launch_detected) {  //hasn't launched, keep updating Ground level
      // Check if we need to correct for drift
      dif_alt = current_alt - gnd_alt;
      if (abs(dif_alt) > DRIFT_THRESHOLD) {
        gnd_alt = altRA.getAverage();  // Reset groundlevel  using running average
        // i think this is redundant, but leaving for double check...
        //dif_alt = current_alt - gnd_alt;  // reset current height to gnd so it doesn't jummp as gnd changed.
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
    } else {
      dif_alt = current_alt - gnd_alt;  // setting dif_alt each loop
    }

    if (launch_detected && !apogee_detected) {              // flying up
      if (altRA.getMax() - altRA.getAverageLast(6) > 50) {  // if height is consistently less than max, apogee happened
        apogee_detected = true;
        apogeeTime = millis();
      }
    }

    if (apogee_detected) {                             // has turned around
      if (millis() - apogeeTime > chuteDelay * 100) {  //chute delay is in 100ths of a sec so 1000 * 100 = 100000 ms = 10 s
        latchServoPosition = LATCH_OPEN;
      }
    }

    //LOGGING

    //fill datastring with logging data:
    //FlightNum, seconds, altitude(ft), rate(m/s), ground level (ft), max(ft), brake%, launch, apogee, latchsrvo, X, Y, Z(m/s^2), batt(mV)
    sprintf(dataString, "FN%d, %4.3f, %4.2f, %4.2f, %4.2f, %4.2f, %d, %d,%d, %d, %4.1f,%4.1f,%4.1f,%d\n ",
            FLIGHT_NUM, currentTime * .001, dif_alt * M2FEET, rate, gnd_alt * M2FEET, (altRA.getMax() - ini_alt) * M2FEET,
            map(brakeServoPosition, BRAKE_CLOSE, BRAKE_OPEN, 0, 100), launch_detected, apogee_detected, latchServoPosition,
            (float)accel.acceleration.x, (float)accel.acceleration.y, (float)accel.acceleration.z, battery.voltage());

    // fill buffer
    if (i <= 9000) {
      i += sprintf(buffer + i, dataString);
    }

    //write out buffer to card
    if (i > 9000) {
      appendFile(SD, rocketLog, buffer);
      i = 0;
    }
  }

  // MENU
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


    if (millis() - dispUpdateTime > 1000) {
      dispUpdateTime = millis();
      tft.setCursor(5, 0, 4);
      tft.setTextColor(TFT_BLUE, TFT_BLACK);  //blue is actual red
      //topline
      tft.print("V");
      tft.print(versionNumber);
      tft.print(" FN");
      tft.print(FLIGHT_NUM);
      tft.print(" ");
      dtostrf((battery.voltage() / 1000.0), 2, 2, battString);
      tft.println(battString);
      tft.print("v");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      //Alt
      tft.setCursor(0, 1 * PIXELPERLINE, 4);
      tft.print("Alt: ");
      tft.println(dif_alt * M2FEET);
      //Max
      tft.setCursor(0, 2 * PIXELPERLINE, 4);
      tft.print("Max: ");
      tft.print((altRA.getMax() - gnd_alt) * M2FEET);
    }

    if (lcdUpdate && (menuState < 99)) {  // refresh main menu display
      //Target
      tft.setCursor(0, 3 * PIXELPERLINE, 4);
      tft.print("Target: ");
      tft.println(TARGET_ALT);
      //HPA
      tft.setCursor(0, 4 * PIXELPERLINE, 4);
      tft.print("HPA: ");
      tft.println(SEALEVELPRESSURE_HPA);
      //Chute
      tft.setCursor(0, 5 * PIXELPERLINE, 4);
      tft.print("Deploy: ");
      tft.println(chuteDelay);
      //BrakeStatus
      tft.setCursor(0, 7 * PIXELPERLINE, 4);
      tft.print("Brakes: ");
      if (brakeServoPosition == BRAKE_OPEN) {
        tft.print("OPEN");
      } else if (brakeServoPosition == BRAKE_CLOSE) {
        tft.print("SHUT");
      } else {
        tft.print(map(brakeServoPosition, BRAKE_CLOSE, BRAKE_OPEN, 0, 100));
        tft.print("%");
      }
      //LatchStatus
      tft.setCursor(0, 8 * PIXELPERLINE, 4);
      tft.print("Latch: ");
      if (latchServoPosition == LATCH_OPEN) {
        tft.print("OPEN ");
      } else if (latchServoPosition == LATCH_CLOSE) {
        tft.print("CLOSE");
      } else {
        tft.print(map(latchServoPosition, LATCH_OPEN, LATCH_CLOSE, 0, 100));
        tft.print("%");
      }

      tft.setCursor(0, 9 * PIXELPERLINE, 4);
      tft.print("   EDIT   ");

      tft.setCursor(0, 10 * PIXELPERLINE, 4);
      tft.print("   DONE   ");
    }

    if (lcdUpdate && (menuState > 99)) {  // refresh main menu display
      //Target
      tft.setCursor(0, 3 * PIXELPERLINE, 4);
      tft.print("Target: ");
      tft.println(TARGET_ALT);
      //HPA
      tft.setCursor(0, 4 * PIXELPERLINE, 4);
      tft.print("HPA: ");
      tft.println(SEALEVELPRESSURE_HPA);
      //Chute
      tft.setCursor(0, 5 * PIXELPERLINE, 4);
      tft.print("Deploy: ");
      tft.println(chuteDelay);
      //BrakeEndpoints
      tft.setCursor(0, 6 * PIXELPERLINE, 4);
      tft.print("BrkMin:");
      tft.println(BRAKE_CLOSE);
      tft.setCursor(0, 7 * PIXELPERLINE, 4);
      tft.print("BrkMax:");
      tft.println(BRAKE_OPEN);
      //LatchEndpoints
      tft.setCursor(0, 8 * PIXELPERLINE, 4);
      tft.print("LtchMin:");
      tft.println(LATCH_OPEN);
      tft.setCursor(0, 9 * PIXELPERLINE, 4);
      tft.print("LtchMax:");
      tft.println(LATCH_CLOSE);
      tft.setCursor(0, 10 * PIXELPERLINE, 4);
      tft.print("   SAVE   ");
    }

    switch (menuState) {
      case 0:                             // main, no hilite
        if (switchClick) menuState = 20;  // jump to highlite w/ click
        break;

      case 10:  // wraparound to end of main
        menuState = 50;
        break;

      case 20:  //Brake Operate
        tft.setCursor(0, 7 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("Brakes: ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (brakeServoPosition == BRAKE_OPEN) {
          tft.print("OPEN");
        } else if (brakeServoPosition == BRAKE_CLOSE) {
          tft.print("SHUT");
        } else {
          tft.print(map(brakeServoPosition, 1220, 1925, 0, 100));
          tft.print("%");
        }
        if (switchClick) menuState = menuState + 5;
        menuState = menuState + 10 * knobInc;
        break;
      case 25:
        tft.setCursor(0, 7 * PIXELPERLINE, 4);
        tft.print("Brakes: ");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        if (brakeServoPosition == BRAKE_OPEN) {
          tft.print("OPEN ");
        } else if (brakeServoPosition == BRAKE_CLOSE) {
          tft.print("SHUT");
        } else {
          tft.print(map(brakeServoPosition, 1220, 1925, 0, 100));
          tft.print("%");
        }
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain

        if (knobInc) {
          switch (knobInc) {
            case 1:
              brakeServoPosition = BRAKE_OPEN;
              break;
            case -1:
              brakeServoPosition = BRAKE_CLOSE;
              break;
          }
        }
        if (switchClick) menuState = menuState - 5;
        break;

      case 30:  //Latch Operate
        tft.setCursor(0, 8 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("Latch: ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (latchServoPosition == LATCH_OPEN) {
          tft.print("OPEN");
        } else if (latchServoPosition == LATCH_CLOSE) {
          tft.print("CLOSE");
        } else {
          tft.print(map(latchServoPosition, 2000, 1000, 0, 100));
          tft.print("%");
        }
        if (switchClick) menuState = menuState + 5;
        menuState = menuState + 10 * knobInc;
        break;
      case 35:
        tft.setCursor(0, 8 * PIXELPERLINE, 4);
        tft.print("Latch: ");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        if (latchServoPosition == LATCH_OPEN) {
          tft.print("OPEN ");
        } else if (latchServoPosition == LATCH_CLOSE) {
          tft.print("CLOSE");
        } else {
          tft.print(map(latchServoPosition, 2000, 1000, 0, 100));
          tft.print("%");
        }
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (knobInc) {
          switch (knobInc) {
            case 1:
              latchServoPosition = LATCH_CLOSE;
              break;
            case -1:
              latchServoPosition = LATCH_OPEN;
              break;
          }
        }
        if (switchClick) menuState = menuState - 5;
        break;

      case 40:  //EDIT
        tft.setCursor(0, 9 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("   EDIT   ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        menuState = menuState + 10 * knobInc;
        if (switchClick) menuState = 110;  //Click to edit
        break;

      case 50:  //DONE
        tft.setCursor(0, 10 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("   DONE   ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        menuState = menuState + 10 * knobInc;
        if (switchClick) menuState = 0;  //Click to go unhilited main
        break;

      case 60:  // wraparound to top of main
        menuState = 20;
        break;

      case 100:  // wraparound to end of edit
        menuState = 180;
        break;

      case 110:                                  // Target
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.setCursor(0, 3 * PIXELPERLINE, 4);
        tft.print("Target: ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        tft.println(TARGET_ALT);
        if (switchClick) menuState = menuState + 5;
        menuState = menuState + 10 * knobInc;
        break;
      case 115:
        tft.setCursor(0, 3 * PIXELPERLINE, 4);
        tft.print("Target: ");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.println(TARGET_ALT);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (switchClick) menuState = menuState - 5;
        TARGET_ALT = TARGET_ALT + knobInc;
        break;

      case 120:                                  // HPA
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.setCursor(0, 4 * PIXELPERLINE, 4);
        tft.print("HPA: ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        tft.println(SEALEVELPRESSURE_HPA);
        if (switchClick) menuState = menuState + 5;
        menuState = menuState + 10 * knobInc;
        break;
      case 125:
        tft.setCursor(0, 4 * PIXELPERLINE, 4);
        tft.print("HPA: ");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.println(SEALEVELPRESSURE_HPA);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (switchClick) menuState = menuState - 5;
        SEALEVELPRESSURE_HPA = SEALEVELPRESSURE_HPA + knobInc;
        break;

      case 130:  //deploy
        tft.setCursor(0, 5 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("Deploy: ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        tft.println(chuteDelay);
        if (switchClick) menuState = menuState + 5;
        menuState = menuState + 10 * knobInc;
        break;
      case 135:
        tft.setCursor(0, 5 * PIXELPERLINE, 4);
        tft.print("Deploy: ");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.println(chuteDelay);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (switchClick) menuState = menuState - 5;
        chuteDelay = chuteDelay + knobInc;
        break;

      case 140:  //brakeMin
        tft.setCursor(0, 6 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("BrkMin:");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        tft.println(BRAKE_CLOSE);
        if (switchClick) menuState = menuState + 5;
        menuState = menuState + 10 * knobInc;
        break;
      case 145:
        tft.setCursor(0, 6 * PIXELPERLINE, 4);
        tft.print("BrkMin:");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.println(BRAKE_CLOSE);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (switchClick) menuState = menuState - 5;
        BRAKE_CLOSE = BRAKE_CLOSE + 5 * knobInc;
        break;

      case 150:  //brakeMax
        tft.setCursor(0, 7 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("BrkMax:");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        tft.println(BRAKE_OPEN);
        if (switchClick) menuState = menuState + 5;
        menuState = menuState + 10 * knobInc;
        break;
      case 155:
        tft.setCursor(0, 7 * PIXELPERLINE, 4);
        tft.print("BrkMax:");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.println(BRAKE_OPEN);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (switchClick) menuState = menuState - 5;
        BRAKE_OPEN = BRAKE_OPEN + 5 * knobInc;
        break;

      case 160:  //latchMin
        tft.setCursor(0, 8 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("LtchMin:");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        tft.println(LATCH_OPEN);
        if (switchClick) menuState = menuState + 5;
        menuState = menuState + 10 * knobInc;
        break;
      case 165:
        tft.setCursor(0, 8 * PIXELPERLINE, 4);
        tft.print("LtchMin:");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.println(LATCH_OPEN);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (switchClick) menuState = menuState - 5;
        LATCH_OPEN = LATCH_OPEN + 5 * knobInc;
        break;

      case 170:  //latchMax
        tft.setCursor(0, 9 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("LtchMax:");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        tft.println(LATCH_CLOSE);
        if (switchClick) menuState = menuState + 5;
        menuState = menuState + 10 * knobInc;
        break;
      case 175:
        tft.setCursor(0, 9 * PIXELPERLINE, 4);
        tft.print("LtchMax:");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.println(LATCH_CLOSE);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain
        if (switchClick) menuState = menuState - 5;
        LATCH_CLOSE = LATCH_CLOSE + 5 * knobInc;
        break;


      case 180:  //Save
        tft.setCursor(0, 10 * PIXELPERLINE, 4);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);  //highlite
        tft.print("   SAVE   ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);  //plain

        if (switchClick) menuState = 0;  //Click to save
        {
          if (SD.exists("/target.txt")) SD.remove("/target.txt");
          sprintf(numString, "%05d", TARGET_ALT);
          writeFile(SD, "/target.txt", numString);

          if (SD.exists("/latchopen.txt")) SD.remove("/latchopen.txt");
          sprintf(numString, "%05d", LATCH_OPEN);
          writeFile(SD, "/latchopen.txt", numString);

          if (SD.exists("/latchshut.txt")) SD.remove("/latchshut.txt");
          sprintf(numString, "%05d", LATCH_CLOSE);
          writeFile(SD, "/latchshut.txt", numString);

          if (SD.exists("/brakeopen.txt")) SD.remove("/brakeopen.txt");
          sprintf(numString, "%05d", BRAKE_OPEN);
          writeFile(SD, "/brakeopen.txt", numString);

          if (SD.exists("/brakeshut.txt")) SD.remove("/brakeshut.txt");
          sprintf(numString, "%05d", BRAKE_CLOSE);
          writeFile(SD, "/brakeshut.txt", numString);

          if (SD.exists("/hpa.txt")) SD.remove("/hpa.txt");
          sprintf(numString, "%05d", SEALEVELPRESSURE_HPA);
          writeFile(SD, "/hpa.txt", numString);

          if (SD.exists("/deploydelay.txt")) SD.remove("/deploydelay.txt");
          sprintf(numString, "%05d", chuteDelay);
          writeFile(SD, "/deploydelay.txt", numString);
        }
        menuState = menuState + 10 * knobInc;
        break;

      case 190:           //wraparound
        menuState = 110;  //back to top
        break;

      default:  // state 0
        // if nothing else matches, do the default
        // default is optional
        break;
    }

    lcdUpdate = true;
    if (prevMenuState == menuState) lcdUpdate = false;
    if (abs(prevMenuState - menuState) > 100) {  // wipe if switching menu <-> edit
      tft.fillScreen(TFT_BLACK);
    }
    prevMenuState = menuState;
  }

  //SERIAL
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
      Serial.print(" menuState ");
      Serial.print(menuState);
      Serial.print(" bat: ");
      Serial.print(battery.voltage() / 1000.0);
      Serial.print(" dataString: ");
      Serial.println(dataString);
    }
  }
}
