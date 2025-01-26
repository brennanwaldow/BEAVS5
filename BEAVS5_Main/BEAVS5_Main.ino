/*
Hardware:
---
Computer: Raspberry Pi Pico RP2040
Altimeter: Adafruit BMP390
Accelerometer: Adafruit BNO055 IMU
*/

// -----   Libraries   -----
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"


// Utility

float feet_to_meters(float length) {     // length [feet]
  return length / 3.28084; // [meters]
}

float meters_to_feet(float length) {     // length [meters]
  return length * 3.28084; // [feet]
}


// -----   Global Variables   -----
String BEAVS_version = "5.0.0";

// Initialization
float launch_altitude = 0; // [meters]
float launch_altimeter = 1013.25; // [HPa]
float target_apogee = feet_to_meters(10000.0); // [meters], AGL

// BMP390
#define SEALEVELPRESSURE_HPA (launch_altimeter)
#define BMP_SCK 13   // vvv TEMPORARY PINS: Reassign for PCB layout
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
Adafruit_BMP3XX bmp;

// PID constants
float kp = 6;
float ki = 9;
float kd = 0;

// Flight Computer
float height = 0; // [meters]
float velocity = 0; // [m/s]
float acceleration = 0; // [m/s^2]
long clock_time = 0; // [ms]

/* Flight Phase
-- 0: Preflight Safe
-- 1: Preflight Armed
-- 2: Flight Phase
-- 3: Coast Phase (BEAVing)
-- 4: Descent Phase
*/
int flight_phase = 0;


// -----   Power-On Boot   -----
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  bmp.begin_I2C();

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

// -----   Control Loop   -----
void loop() {
  switch(flight_phase) {
    case 0:
      preflight_loop();
      break;
    case 1:
      ready_loop();
      break;
    case 2:
      flight_loop();
      break;
    case 3:
      coast_loop();
      break;
  }
}


void preflight_loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  if (millis() > 5000) arm();

  collect_telemetry();
}

void ready_loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);

  collect_telemetry();
}

void flight_loop() {
  collect_telemetry();
}

void coast_loop() {
  collect_telemetry();
}



// -----   Phase Changeovers   -----
void arm() {
  // SAFETY PIN REMOVED: Arm BEAVS monitoring and initiate startup
  flight_phase = 1;
}

void disarm() {
  // SAFETY PIN REINSERTED: Return to Disarmed state
  flight_phase = 0;
}

void launch() {
  // ENGINE IGNITION: Arm BEAVS monitoring for cutoff
  flight_phase = 2;
}

void coast() {
  // ENGINE CUTOFF: Deploy BEAVS
  flight_phase = 3;
}

void descend() {
  // APOGEE REACHED: BEAVS safing, PID shutdown
  flight_phase = 4;
}



// -----   Functions   -----

void collect_telemetry() {
  bmp.performReading();

  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  height = altitude - launch_altitude;
}

// Tick PID controller
void PID() {

}

void command_deflection(float deflection) {  // [ratio], 0 to 1

}

// Velocity lookup table
void velocity_lookup() {

}


// Utility functions

// If needed: utility functions for deployment ratio <--> servo degrees <--> c_d
