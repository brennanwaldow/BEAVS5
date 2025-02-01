/*
Hardware:
---
Computer: Raspberry Pi Pico RP2040
Altimeter: Adafruit BMP390
Accelerometer: Adafruit BNO055 IMU
Servo: DS3235
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
#define BMP_SCK 2   // vvv TEMPORARY PINS: Reassign for PCB layout
#define BMP_MOSI 1   // SDA pin
Adafruit_BMP3XX bmp;

// Pins
int servo_pin = 28;

// PID constants
float kp = 1;
float ki = 1;
float kd = 1;

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
enum {PREFLIGHT, ARMED, FLIGHT, COAST, DESCENT};
int flight_phase = PREFLIGHT;


// -----   Power-On Boot   -----
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  bmp.begin_I2C();

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  pinMode(servo_pin, OUTPUT);

  // Ensure servo is stowed
  command_deflection(-1);
}

// -----   Control Loop   -----
void loop() {

  switch(flight_phase) {
    case PREFLIGHT:
      preflight_loop();
      break;
    case ARMED:
      ready_loop();
      break;
    case FLIGHT:
      flight_loop();
      break;
    case COAST:
      coast_loop();
      break;
  }
}


void preflight_loop() {
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(1000);

  if (millis() > 3000) arm();

  collect_telemetry();
}

void ready_loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  // delay(100);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(100);

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
  flight_phase = ARMED;

  command_deflection(1);
  delay(2000);
  command_deflection(0.5);
  delay(1000);
  command_deflection(0);
  delay(3000);
  command_deflection(-1);
}

void disarm() {
  // SAFETY PIN REINSERTED: Return to Disarmed state
  flight_phase = PREFLIGHT;
}

void launch() {
  // ENGINE IGNITION: Arm BEAVS monitoring for cutoff
  flight_phase = FLIGHT;
}

void coast() {
  // ENGINE CUTOFF: Deploy BEAVS
  flight_phase = COAST;
}

void descend() {
  // APOGEE REACHED: BEAVS safing, PID shutdown
  flight_phase = DESCENT;
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

void command_deflection(float deflection) {  // [ratio], 0 (flush) to 1 (full extend); or -1 for full retract (inside tube Inner Diameter)
  int min = 50;
  int max = 180;

  int max_pulse = 2500;
  int min_pulse = 500;
  
  float angle = 0;

  // Remap angle
  if (deflection > -1) angle = deflection * (max - min) + min;

  int x = (max_pulse - min_pulse) * (angle / max) + min_pulse;

  for (int i = 0; i <= 2; i++) {
    // A pulse each 20ms
    digitalWrite(servo_pin, HIGH);
    delayMicroseconds(x);
    digitalWrite(servo_pin, LOW);
    delayMicroseconds(18550);
    // Pulses duration: 500 - 0deg; 1500 - 90deg; 2500 - 180deg
  }
}

// Velocity lookup table
void velocity_lookup() {

}


// Utility functions

// If needed: utility functions for deployment ratio <--> servo degrees <--> c_d
