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
float launch_altitude = 1380; // [meters]; Brothers, OR
float launch_altimeter = 1013.25; // [HPa]
float target_apogee = feet_to_meters(10000.0); // [meters], AGL

// BMP390
#define SEALEVELPRESSURE_HPA (launch_altimeter)
#define BMP_SCK 2   // vvv TEMPORARY PINS: Reassign for PCB layout
#define BMP_MOSI 1   // SDA pin
Adafruit_BMP3XX bmp;

// Servo
int servo_pin = 28;

// PID constants
float kp = 1;
float ki = 1;
float kd = 1;

// Flight Computer
float altitude = 0; // [meters]
float height = 0; // [meters]
float velocity = 0; // [m/s]
float acceleration = 0; // [m/s^2]
long clock_time = 0; // [ms]

float max_height = 0; // [meters]
long max_height_clock = 0; // [ms]

float commanded_angle = 0; // [degrees; 0 to 180]

/* Flight Phase
-- 0: Preflight Safe
-- 1: Preflight Armed
-- 2: Flight Phase
-- 3: Coast Phase (BEAVing)
-- 4: Descent Phase
*/
enum {PREFLIGHT, ARMED, FLIGHT, COAST, OVERSHOOT, DESCENT};
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
    case OVERSHOOT:
      overshoot_loop();
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

  // collect_telemetry();
  get_trolled_idiot();
  calculate_telemetry();
}

void ready_loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  // delay(100);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(100);

  // collect_telemetry();
  get_trolled_idiot();
  calculate_telemetry();

  if (acceleration > 10) launch();
}

void flight_loop() {
  // collect_telemetry();
  get_trolled_idiot();
  calculate_telemetry();

  if (acceleration < 5) coast();
}

void coast_loop() {
  // collect_telemetry();
  get_trolled_idiot();
  calculate_telemetry();

  PID();

  if (height > target_apogee) overshoot();

  if (max_height > height && (millis() - max_height_clock) > 500) {
    descend();
  }
}

void overshoot_loop() {
  // collect_telemetry();
  get_trolled_idiot();
  calculate_telemetry();

  if (max_height > (height + 50)) descend();

  if (max_height > height && (millis() - max_height_clock) > 500) {
    descend();
  }
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

  // Recalibrate launch ground level to current altitude?
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
  command_deflection(1);
}

void overshoot() {
  // APOGEE OVERSHOT: Last full extension regime on airbrake to minimize further overshoot
  flight_phase = OVERSHOOT;
  command_deflection(1);
}

void descend() {
  // APOGEE REACHED: BEAVS safing, PID shutdown
  flight_phase = DESCENT;
  command_deflection(0);
}



// -----   Functions   -----

void collect_telemetry() {
  bmp.performReading();

  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  height = altitude - launch_altitude;
}

void calculate_telemetry() {
  if (height > max_height) {
    max_height = height;
    max_height_clock = millis();
  }
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
  commanded_angle = angle;

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


// Spoof telemetry data with real-time simulated flight
// RUDIMENTARY SIMULATION: ONLY for validating logic behavior, NOT pid response
void get_trolled_idiot() {
  // Apogee: 3500m (11500 ft)

  // Cd: ~ 0.6
  // T = 3000 N
  // Drag force max = 900 N at burnout
  // Max velocity = 350 m/s at burnout
  // Max acceleration = 120 m/s^2 at halfway burn
  // Max deceleration = -45 m/s^2 at burnout
  // Mass = 27.6 kg, mass at burnout = 22.9 kg, D = 15.6 cm

  // Motor cutout at 4 s, apogee at 25

  long launch_clock = millis() - 10000;

  float speed_of_sound = (-0.0039042 * altitude) + 340.3;
  float Mach = velocity / speed_of_sound;
  float Cd = (0.0936073 * (Mach * Mach * Mach)) + (-0.0399526 * (Mach * Mach)) + (0.0455436 * Mach) + 0.582895;

  float mass = max(22.9, (-0.00112172 * launch_clock) + 27.6);

  double dt = (millis() - clock_time) / 1000.0;

  // Launch
  // if (clock_time > 10000 && clock_time < 14000) acceleration = -9.81 + (3000 / 27.6);
  // else if (clock_time > 10000) acceleration = -9.81;

  float thrust = 0;

  if (launch_clock > 0 && launch_clock <= 1590) thrust = (0.658927 * launch_clock) + 2446.50196;
  else if (launch_clock > 1590 && launch_clock <= 3240) thrust = (-0.373939 * launch_clock) + 4105.56364;
  else if (launch_clock > 3240 && launch_clock <= 4190) thrust = (-3.04632 * launch_clock) + 12764.0632;
  // else if (launch_clock > 4190) return;
  
  if (launch_clock > 0) acceleration = -9.81 + (thrust / mass);

  // Drag
  int dir = 1;
  if (velocity < 0) dir = -1;
  float Fd = (0.5 * 1.225 * velocity * velocity * Cd * (0.0191)) * dir;
  acceleration = acceleration - (Fd / mass);

  float dv = acceleration * dt;

  if (launch_clock > 0) velocity = velocity + dv;

  float dh = velocity * dt;
  height = height + dh;
  if (height < 0) height = 0;

  Serial.print((float) launch_clock / 1000.0);
  Serial.print(" ");
  Serial.print(acceleration);
  Serial.print(" ");
  Serial.print(velocity);
  Serial.print(" ");


  Serial.print(height);
  Serial.print(" ");
  Serial.print(Fd);
  Serial.print(" ");
  Serial.print(thrust);
  Serial.print(" ");
  Serial.println(flight_phase);
  
  clock_time = millis();

  delay(100);
}


// Utility functions

// If needed: utility functions for deployment ratio <--> servo degrees <--> c_d
