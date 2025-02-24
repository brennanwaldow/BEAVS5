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
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// Utility

float feet_to_meters(float length) {     // length [feet]
  return length / 3.28084; // [meters]
}

float meters_to_feet(float length) {     // length [meters]
  return length * 3.28084; // [feet]
}

float inhg_to_hpa(float pressure) {     // pressure [inches mercury]
  return pressure * 33.863889532611; // [ HPa ]
}


// -----   Global Variables   -----
String BEAVS_version = "5.0.0";

// Initialization
enum { SIM, FIELD };
int BEAVS_mode = SIM;

enum { SEA_LEVEL = 0, BROTHERS_OR = 1380 };
float launch_altitude = SEA_LEVEL; // [meters]
float launch_altimeter = inhg_to_hpa(30.49); // [HPa]
float target_apogee = feet_to_meters(10000.0); // [meters], AGL

// BMP390
#define WIRE Wire
#define SEALEVELPRESSURE_HPA (launch_altimeter)
Adafruit_BMP3XX bmp;

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Servo
int servo_pin = 28;

// PID constants
float kp = 0.2000e-04;
float ki = 1.250e-09;
float kd = 7.500e-05;

float target_velocity = 0;
float u = 0;

float error1 = 0;
float error2 = 0;
float error3 = 0;

// Flight Computer
float altitude = launch_altitude; // [meters]
float height = 0; // [meters]
double velocity = 0; // [m/s]
double acceleration = 0; // [m/s^2]
long clock_time = 0; // [ms]
long pid_clock_time = 0; // [ms]

float max_height = 0; // [meters]
long max_height_clock = 0; // [ms]

float commanded_angle = 0; // [degrees; 0 to 180]
float virtual_angle = 0; // [degrees; 0 to 180]

/* Flight Phase
-- 0: Preflight Safe
-- 1: Preflight Armed
-- 2: Flight Phase
-- 3: Coast Phase (BEAVing)
-- 4: Descent Phase
*/
enum { PREFLIGHT, ARMED, FLIGHT, COAST, OVERSHOOT, DESCENT };
int flight_phase = PREFLIGHT;


// -----   Power-On Boot   -----
void setup() {
  Serial.begin(115200);
  // Wait for serial to initialize
  delay(500);

  Serial.println("Starting up hardware...");

  Wire.begin();

  if (!bmp.begin_I2C(0x77)) Serial.println("BMP390 invalid!");

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (!bno.begin()) Serial.println("BNO055 invalid!");
  bno.setExtCrystalUse(true);

  pinMode(servo_pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Ensure servo is fully stowed
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
    case DESCENT:
      descend_loop();
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
  calculate_telemetry();
}

void ready_loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  // delay(100);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(100);

  collect_telemetry();
  calculate_telemetry();

  if (acceleration > 10) launch();
}

void flight_loop() {
  collect_telemetry();
  calculate_telemetry();

  if (acceleration < 5) coast();
}

void coast_loop() {
  collect_telemetry();
  calculate_telemetry();

  PID();

  command_deflection(u);

  if (height > target_apogee) overshoot();

  // TODO the altitude reading is TOO NOISY for this cutoff: will have to add additional conditions
  if (max_height > height && (millis() - max_height_clock) > 500) {
    descend();
  }
}

void overshoot_loop() {
  collect_telemetry();
  calculate_telemetry();

  if (max_height > (height + 50)) descend();

  if (max_height > height && (millis() - max_height_clock) > 500) {
    descend();
  }
}

void descend_loop() {
  // collect_telemetry();
  // calculate_telemetry();
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
  // command_deflection(1);
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
  if (BEAVS_mode == SIM) get_trolled_idiot();
  else {
    // BMP390
    if (!bmp.performReading()) Serial.println("BMP390 Reading failed!");

    float prev_altitude = altitude;
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    // Serial.println(altitude);

    double dt = (micros() - clock_time) / (double) 1000000;
    velocity = (altitude - prev_altitude) / dt;
    clock_time = micros();
    
    // BNO055
    sensors_event_t accelerometer;
    bno.getEvent(&accelerometer, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // TODO: 3-axis -> magnitude?
    sensors_event_t* event = &accelerometer;
    event->type == SENSOR_TYPE_ACCELEROMETER;
    acceleration = -(event->acceleration.z) + gravity(altitude);

    Serial.print(altitude);
    Serial.print(" ");
    Serial.print(prev_altitude);
    Serial.print(" ");
    Serial.print(dt);
    Serial.print(" ");
    Serial.println(velocity);
    delay(50);
  }
}

void calculate_telemetry() {
  height = altitude - launch_altitude;
  if (height > max_height) {
    max_height = height;
    max_height_clock = millis();
  }
}

// Tick PID controller
void PID() {
  double dt = (micros() - pid_clock_time) / (double) 1000;

  target_velocity = velocity_lookup();

  error3 = error2;
  error2 = error1;
  error1 = target_velocity - velocity;
  // Serial.println(target_velocity);

  u = u + ((kp + (ki * dt) + (kd / dt)) * error1) + abs((-kp - (2*kd / dt)) * error2) + ((kd / dt) * error3);

  pid_clock_time = micros();
}

void command_deflection(float deflection) {  // [ratio], 0 (flush) to 1 (full extend); or -1 for full retract (inside tube Inner Diameter)
  int min = 50;
  int max = 180;

  int max_pulse = 2500;
  int min_pulse = 500;
  
  float angle = 0;

  // Remap angle
  if (deflection >= 0) angle = deflection * (max - min) + min;
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
float velocity_lookup() {
  // Shamelessly stolen polynomial constants from BEAVS4
  // THESE ARE NOT FOR 2025 ROCKET DRAG
  // TODO UPDATE WHEN OPENROCKET FINALIZED
  float a = -2.197790209276072e-9;
  float b =  1.424454885385808e-6;
  float c = -2.425899535946396e-4;
  float d = -0.031992842779187;
  float e = -0.261849939656465;

  return (a * pow(height, 4)) + (b * pow(height, 3)) + (c * pow(height, 2)) + (d * height) + e;
}


// Spoof telemetry data with real-time simulated flight
// RUDIMENTARY SIMULATION: ONLY for validating logic behavior, NOT pid response
void get_trolled_idiot() {
  // Apogee: 3493m (11459 ft)

  // Cd: ~ 0.6
  // T = 3000 N
  // Drag force max = 900 N at burnout
  // Max velocity = 350 m/s at burnout
  // Max acceleration = 120 m/s^2 at halfway burn
  // Max deceleration = -45 m/s^2 at burnout
  // Mass = 27.6 kg, mass at burnout = 22.9 kg, D = 15.6 cm

  // Motor cutout at 4 s, apogee at 25.03

  long launch_clock = millis() - 10000;

  float speed_of_sound = (-0.0039042 * altitude) + 340.3;
  float Mach = abs(velocity) / speed_of_sound;
  float Cd_rocket = (0.0936073 * (Mach * Mach * Mach)) + (-0.0399526 * (Mach * Mach)) + (0.0455436 * Mach) + 0.582895;
  float air_density = (-6.85185 * (pow(10, -14)) * pow(altitude, 3)) + (4.30675 * (pow(10, -9)) * pow(altitude, 2)) + (-0.0001176 * altitude) + 1.22499;

  float mass = 22.863;
  if (launch_clock >= 0 && launch_clock < 4260) mass = (1.74432 * pow(10, -7) * pow(launch_clock, 2)) + (-0.00191159 * launch_clock) + 27.87811;

  double dt = (micros() - clock_time) / (double) 1000000;

  // Launch
  float thrust = 0;

  if (launch_clock > 0 && launch_clock <= 1590) thrust = (0.662162 * launch_clock) + 2458.16216;
  else if (launch_clock > 1590 && launch_clock <= 3240) thrust = (-0.000111179 * pow(launch_clock, 2)) + (0.171212 * launch_clock) + 3507.36323;
  else if (launch_clock > 3240 && launch_clock <= 4190) thrust = (-3.04632 * launch_clock) + 12764.0632;
  
  if (launch_clock > 0) acceleration = -(gravity(altitude)) + (thrust / mass);

  // Drag
  int dir = 1;
  if (velocity < 0) dir = -1;

  // TODO: is this right? it seems low for 6in diameter
  float A_ref = 0.019009;
  float virtual_deflection = max((virtual_angle - 50) / (180 - 50), 0);
  // TODO: Caliper time (measure the Metalbeav)
  float A_beavs = (((1.8 / 12) * (2.490 / 12)) * 2) * virtual_deflection;
  // TODO: Polyfit from Ansys Fluent god help us
  float Cd_beavs = 4.8 * (sqrt(A_beavs / A_ref));
  float Cd = Cd_rocket + (Cd_beavs * (A_beavs / A_ref));

  float Fd = (0.5 * air_density * (velocity * velocity) * Cd * A_ref) * dir;
  acceleration = acceleration - (Fd / mass);

  double dv = acceleration * dt;

  if (launch_clock > 0) velocity = velocity + dv;

  double dh = velocity * dt;
  altitude = altitude + dh;
  if (altitude < launch_altitude) altitude = launch_altitude;
  
  // Smoothly adjust physical blade angle based on servo speed
  // TODO: Slowmo stopwatch for precise time after integrating on Metalbeav for speed under torque loading
  if (commanded_angle > virtual_angle) virtual_angle = virtual_angle + min(commanded_angle - virtual_angle, (180.0 / 1.0) * dt);
  else if (commanded_angle < virtual_angle) virtual_angle = virtual_angle + max(commanded_angle - virtual_angle, -(180.0 / 1.0) * dt);

  Serial.print((float) launch_clock / 1000.0);
  Serial.print(" ");
  Serial.print(acceleration);
  Serial.print(" ");
  Serial.print(velocity);
  Serial.print(" ");


  Serial.print(height);
  Serial.print(" ");
  // Serial.print(Fd);
  // Serial.print(" ");
  // Serial.print(thrust);
  // Serial.print(" ");
  // Serial.print(Mach);
  // Serial.print(" ");
  Serial.print(Cd_beavs);
  Serial.print(" ");
  Serial.print(Cd_rocket);
  Serial.print(" ");
  Serial.print(u);
  Serial.print(" ");
  Serial.println(virtual_angle);
  // Serial.print(" ");
  // Serial.println(flight_phase);

  // Serial.print((float) launch_clock / 1000.0);
  // Serial.print(",");
  // Serial.print(acceleration);
  // Serial.print(",");
  // Serial.print(velocity);
  // Serial.print(",");
  // Serial.println(height);
  
  clock_time = micros();

  delay(75);
}


// Utility functions

float gravity(float altitude) {          // altitude [meters]
  return (-0.00000325714 * altitude) + 9.80714; // acceleration magnitude [m/s^2]
}


// If needed: utility functions for deployment ratio <--> servo degrees <--> c_d
