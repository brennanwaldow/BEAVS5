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
enum { STOWED, ACTIVE };

// SET TO FIELD BEFORE FLIGHT
int BEAVS_mode = SIM;
int BEAVS_control = ACTIVE;

enum { SEA_LEVEL = 0, BROTHERS_OR = 1380 };
float launch_altitude = SEA_LEVEL; // [meters]
float launch_altimeter = inhg_to_hpa(30.49); // [HPa]
float target_apogee = feet_to_meters(10000.0); // [meters], AGL

// Simulation only
float launch_angle = 0; // [degrees], from vertical
float perpendicular_acceleration = 0; // [m/s^2]
float perpendicular_velocity = 0; // [m/s]

// BMP390
#define WIRE Wire
#define SEALEVELPRESSURE_HPA (launch_altimeter)
Adafruit_BMP3XX bmp;

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Servo
int servo_pin = 28; // GPIO 28 / Physical Pin 34

// PID constants
// float kp = 0.2000e-04;
// float ki = 1.250e-09;
// float kd = 7.500e-05;

// float kp = 0.8000e-04;
// float ki = 1.250e-09;
// float kd = 7.500e-05;

// double kp = 8.000e-09;
// double ki = 2.500e-06;
// double kd = 0;
// TODO: perfect performance at Kd = 0?? investigate further, especially for noise damping

double kp = 5.600e-05;
double ki = 2.500e-06;
double kd = 4.688e-05;


float target_velocity = 0; // [m/s]
float u = 0; // [ratio], 0 to 1

float error1 = 0; // [m/s]
float error2 = 0; // [m/s]
float error3 = 0; // [m/s]

// Flight Computer
float altitude = launch_altitude;  // [meters]
float height = 0;                  // [meters]
double velocity = 0;               // [m/s]
double acceleration = 0;           // [m/s^2]
long clock_time = 0;               // [ms]
long curr_time = 0;                // [micro s]
long pid_clock_time = 0;           // [ms]

float max_height = 0;              // [meters]
long max_height_clock = 0;         // [ms]

float commanded_angle = 0;         // [degrees; 0 to 270]
float virtual_angle = 0;           // [degrees; 0 to 270]

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
  curr_time = micros();

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

  clock_time = curr_time;

  delay(5);
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

  if (BEAVS_control == ACTIVE) command_deflection(u);

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

  if (BEAVS_control == STOWED) {
    command_deflection(0);
  } else if (BEAVS_mode == FIELD) {
    command_deflection(1);
    delay(2000);
    command_deflection(0.5);
    delay(1000);
    command_deflection(0);
  } else if (BEAVS_mode == SIM) {
    command_deflection(1);
    delay(1500);
    command_deflection(0);
    delay(1500);
    command_deflection(0.5);
    delay(1000);
    command_deflection(0);
    delay(1000);
  }

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
}

void overshoot() {
  // APOGEE OVERSHOT: Last full extension regime on airbrake to minimize further overshoot
  flight_phase = OVERSHOOT;
  if (BEAVS_control == ACTIVE) command_deflection(1);
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
  // long curr_time = micros();
  double dt = (curr_time - clock_time) / (double) 1000;

  target_velocity = velocity_lookup();

  error3 = error2;
  error2 = error1;
  error1 = velocity - target_velocity;

  u = u + (double) (((kp + (ki * dt) + (kd / dt)) * error1) + abs((-kp - (2*kd / dt)) * error2) + ((kd / dt) * error3));
  if (u > 1) u = 1;
  if (u < 0) u = 0;
  // Serial.print(dt);
  // Serial.print(" ");
  // Serial.print(error1);
  // Serial.print(" ");
  // Serial.print(error2);
  // Serial.print(" ");
  // Serial.print(error3);
  // Serial.print(" ");
  // Serial.println(((kp + (ki * dt) + (kd / dt)) * error1) + abs((-kp - (2*kd / dt)) * error2) + ((kd / dt) * error3), 5);

  // pid_clock_time = curr_time;

  if (BEAVS_control == STOWED) u = 0;
}

void command_deflection(float deflection) {  // [ratio], 0 (flush) to 1 (full extend); or -1 for full retract (inside tube Inner Diameter)
  float min_def = 8.86;
  float max_def = 150.0;
  float max_deg = 270.0;

  int max_pulse = 2500;
  int min_pulse = 500;

  float angle = 0;

  // Remap angle
  if (deflection >= 0) angle = deflection * (max_def - min_def) + min_def;
  if (angle > max_def) angle = max_def;
  commanded_angle = angle;

  int x = (max_pulse - min_pulse) * (angle / max_deg) + min_pulse;

  for (int i = 0; i <= 2; i++) {
    // A pulse each 20ms

    digitalWrite(servo_pin, HIGH);
    delayMicroseconds(x);
    digitalWrite(servo_pin, LOW);
    delayMicroseconds(18550);

    // Pulses duration: 500 - 0deg; 1500 - 135deg; 2500 - 270deg
    // TODO (i think. we must verify)
  }
}

// Velocity lookup table
float velocity_lookup() {
  // These constants are obtained from ../Utilities/lookup_table.py using the OpenRocket simulation
  double consts[] = {
    -1.97968329189789e-43,
    4.752793902659054e-39,
    -5.021845918026457e-35,
    3.0040180625396656e-31,
    -1.0557454943452812e-27,
    1.7176847161272918e-24,
    2.629175167886028e-21,
    -2.429214233273044e-17,
    7.548454166456636e-14,
    -1.468172319277919e-10,
    1.9752220286824027e-07,
    -0.00018773406093033337,
    0.12431823221950207,
    -54.710515563673326,
    14398.409844721882,
    -1715011.8987151487
  };

  int poly_order = 15;
  double result = 0;

  // pain
  for (int i = 0; i < poly_order + 1; i++) {
    result = result + ((double) pow(height, poly_order - i) * consts[i]);
  }

  // Range of polynomial validity: Burnout to apogee
  if (height < 841.511) return 296.503788;
  if (height > target_apogee) return 0;
  return (float) result;
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
  float Cd_rocket = get_Cd(Mach);
  float air_density = (-6.85185 * (pow(10, -14)) * pow(altitude, 3)) + (4.30675 * (pow(10, -9)) * pow(altitude, 2)) + (-0.0001176 * altitude) + 1.22499;

  float mass = get_mass(launch_clock);

  // double curr_time = micros();
  double dt = (curr_time - clock_time) / (double) 1000000;

  // Launch
  float thrust = get_thrust(launch_clock);

  // if (launch_clock > 0 && launch_clock <= 1590) thrust = (0.662162 * launch_clock) + 2458.16216;
  // else if (launch_clock > 1590 && launch_clock <= 3240) thrust = (-0.000111179 * pow(launch_clock, 2)) + (0.171212 * launch_clock) + 3507.36323;
  // else if (launch_clock > 3240 && launch_clock <= 4190) thrust = (-3.04632 * launch_clock) + 12764.0632;

  // Modulate thrust to simulate performance deviation in reality
  thrust = thrust * 1;

  if (launch_clock > 0) {
    acceleration = -(gravity(altitude) * cos(degrees_to_radians(launch_angle))) + (thrust / mass);
    perpendicular_acceleration = (gravity(altitude) * sin(degrees_to_radians(launch_angle)));
  }

  // Drag
  int dir = 1;
  if (velocity < 0) dir = -1;

  // TODO: Update for new Outer Diameter when openrocket finalized
  float A_ref = 0.019113;
  float virtual_deflection = max((virtual_angle - 8.86) / (150 - 8.86), 0);
  float A_beavs = ((feet_to_meters(1.632 / 12) * feet_to_meters(2.490 / 12)) * 2) * virtual_deflection;
  // TODO: Polyfit from Ansys Fluent god help us
  float Cd_beavs = 4.8 * (sqrt(A_beavs / A_ref));
  float Cd = Cd_rocket + (Cd_beavs * (A_beavs / A_ref));

  float Fd = (0.5 * air_density * (velocity * velocity) * Cd * A_ref) * dir;
  acceleration = acceleration - (Fd / mass);

  double dv = acceleration * dt;
  double dv_perpendicular = perpendicular_acceleration * dt;

  if (launch_clock > 0) {
    velocity = velocity + dv;
    perpendicular_velocity = perpendicular_velocity + dv_perpendicular;
  }

  // From rotated coordinate system to vertical coordinate system
  double dh = ((velocity * dt) * cos(degrees_to_radians(launch_angle))); // - (perpendicular_velocity * sin(degrees_to_radians(launch_angle)));
  altitude = altitude + dh;
  if (altitude < launch_altitude) altitude = launch_altitude;

  // Fix flight conditions to post-launch without simulating burn:
  // if (launch_clock < (4.16 - 0.05) * 1000) {
  //   altitude = 846.352;
  //   acceleration = 0;
  //   velocity = 339.186;
  // } else {
  //   flight_phase = COAST;
  // }

  // Smoothly adjust physical blade angle based on measured servo speed
  if (commanded_angle > virtual_angle) virtual_angle = virtual_angle + min(commanded_angle - virtual_angle, (150.0 / 1.41) * dt);
  else if (commanded_angle < virtual_angle) virtual_angle = virtual_angle + max(commanded_angle - virtual_angle, -(150.0 / 1.41) * dt);

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
  Serial.print(get_Fd_BEAVS(velocity, Cd_beavs, A_beavs, air_density));
  Serial.print(" ");
  Serial.print(mass * 9.81);
  Serial.print(" ");
  Serial.print(thrust);
  Serial.print(" ");
  Serial.print(Mach);
  Serial.print(" ");
  Serial.print(Cd_beavs);
  Serial.print(" ");
  Serial.print(Cd_rocket);
  Serial.print(" ");
  Serial.print(Cd);
  Serial.print(" ");
  Serial.print(u, 5);
  Serial.print(" ");
  Serial.print(target_velocity);
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
}


// Utility functions

float gravity(float altitude) {          // altitude [meters]
  // Range of polynomial validity
  if (altitude < 0.0008627) return 9.79211050803139;
  if (altitude > 5554.943) return 9.774974605475466;

  // These constants are obtained from ../Utilities/gravity_extractor.py using the OpenRocket simulation
  double consts[] = {
    -2.1835147818201505e-21,   // P1
    2.3723692527332454e-17,   // P2
    -7.272030320822667e-14,   // P3
    8.20700344938699e-12,   // P4
    -2.873840586662284e-06,   // P5
    9.792110510510653,   // P6
  };

  int poly_order = 5;
  double result = 0;

  // pain
  for (int i = 0; i < poly_order + 1; i++) {
    result = result + ((double) pow(altitude, poly_order - i) * consts[i]);
  }

  return (float) result;
}

float get_Cd(float mach) {
  double result = 0;

  if (mach < 0.102) {
    // Plateau drag curve at low speed on launch
    if (height < 150) return 0.58486;

    // Apogee regime:

    // Lower bound
    if (mach < 0.019) return 0.68759;

    // These constants are obtained from ../Utilities/drag_curve.py using the OpenRocket simulation
    double consts[] = {
      3.943998910536891e+21,    // P1
      -3.474565463793999e+21,    // P2
      1.4043677485773582e+21,    // P3
      -3.451650801998561e+20,    // P4
      5.764199260473382e+19,    // P5
      -6.922175385568618e+18,    // P6
      6.170037697792379e+17,    // P7
      -4.153146736340778e+16,    // P8
      2126750824405024.0,    // P9
      -82788507607171.47,    // P10
      2428099426082.4634,    // P11
      -52657007161.88133,    // P12
      816908342.3766693,    // P13
      -8554948.870212069,    // P14
      54053.58148892024,    // P15
      -154.56717901831954,    // P16
    };

    int poly_order = 15;

    // pain
    for (int i = 0; i < poly_order + 1; i++) {
      result = result + ((double) pow(mach, poly_order - i) * consts[i]);
    }
  }

  // Polynomial upper bounds
  if (mach > 1.196) return 0.643958;

  // Subsonic regime:
  if (mach > 0.1 && mach < 1) {
    // These constants are obtained from ../Utilities/drag_curve.py using the OpenRocket simulation
    double consts[] = {
      -155187.47240576352,  // P1
      1182842.28739123,     // P2
      -4076124.270327396,   // P3
      8396869.561414396,    // P4
      -11518096.443543786,  // P5
      11088080.449477628,   // P6
      -7687620.83584739,    // P7
      3872843.2085753293,   // P8
      -1409280.065684936,   // P9
      361534.3709655419,    // P10
      -61669.144616050056,  // P11
      5933.480714665612,    // P12
      -79.88202829796315,   // P13
      -50.317225603655366,  // P14
      5.33300787823518,     // P15
      0.4051088969787709,   // P16
    };

    int poly_order = 15;

    // pain
    for (int i = 0; i < poly_order + 1; i++) {
      result = result + ((double) pow(mach, poly_order - i) * consts[i]);
    }
  }

  // Transonic regime:
  if (mach > 1) {
    double consts[] = {
      328346.5670532591,    // P1
      -2236860.928719251,   // P2
      5748733.5980847785,   // P3
      -5681869.204619244,   // P4
      -1899437.479497596,   // P5
      7815582.122333516,    // P6
      -1232208.6779305444,  // P7
      -9807684.684020657,   // P8
      11258163.949775018,   // P9
      -5214225.478380677,   // P10
      921460.8814625319,    // P11
    };

    int poly_order = 10;

    // pain
    for (int i = 0; i < poly_order + 1; i++) {
      result = result + ((double) pow(mach, poly_order - i) * consts[i]);
    }
  }

  return (float) result;
}

float get_thrust(float time) { // [ms]
  // Range of polynomial validity
  if (time < 0) return 0;
  if (time > 4188) return 0;

  // These constants are obtained from ../Utilities/thrust_extractor.py using the OpenRocket simulation
  double consts[] = {
    1.4788401950481751e-60,
    -5.430596612041015e-56,
    8.886785448930557e-52,
    -8.403664526635023e-48,
    4.880248073231894e-44,
    -1.5684417727986281e-40,
    2.127850290614022e-38,
    2.6254116851458864e-33,
    -1.5419314149617205e-29,
    5.285963613417249e-26,
    -1.2638702503695773e-22,
    2.2174213731433413e-19,
    -2.899620650529627e-16,
    2.823612201552925e-13,
    -2.022677406801039e-10,
    1.0417055113804496e-07,
    -3.72206730108048e-05,
    0.00874506638494612,
    -1.242852018405207,
    93.26899190217185,
    -168.71393439284608
  };

  int poly_order = 20;
  double result = 0;

  // pain
  for (int i = 0; i < poly_order + 1; i++) {
    result = result + ((double) pow(time, poly_order - i) * consts[i]);
  }

  return (float) result;
}

float get_mass(float time) { // [ms]
  // Range of polynomial validity
  if (time < 0) return 27.64;
  if (time > 10000) return 22.86;

  // These constants are obtained from ../Utilities/mass_extractor.py using the OpenRocket simulation
  double consts[] = {
    8.668381669216739e-60,
    -1.5955042856385969e-54,
    1.3128951896033921e-49,
    -6.361429030677903e-45,
    2.0103053474988076e-40,
    -4.3290154623097466e-36,
    6.425240565797594e-32,
    -6.45375900533127e-28,
    4.097728297876439e-24,
    -1.2906046395728657e-20,
    -1.275797911407393e-17,
    2.5778237964289546e-13,
    -7.679104125706069e-10,
    7.566678857489254e-07,
    -0.001416982837409509,
    27.69946400632899
  };

  int poly_order = 15;
  double result = 0;

  // pain
  for (int i = 0; i < poly_order + 1; i++) {
    result = result + ((double) pow(time, poly_order - i) * consts[i]);
  }

  return (float) result;
}

float get_Fd_BEAVS(float velocity, float Cd_BEAVS, float A_BEAVS, float air_density) {
  return 0.5 * air_density * (velocity * velocity) * Cd_BEAVS * A_BEAVS;
}

float degrees_to_radians(float degrees) {
  return (degrees * 3.1415926535897932384626433832795) / 180;
}


// If needed: utility functions for deployment ratio <--> servo degrees <--> c_d
