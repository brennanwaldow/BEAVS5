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

// TODO: SET TO FIELD/ACTIVE BEFORE FLIGHT
int BEAVS_mode = SIM;
int BEAVS_control = STOWED;

enum { SEA_LEVEL = 0, BROTHERS_OR = 1380 };
float launch_altitude = SEA_LEVEL; // [meters]
// TODO: Obtain pressure forecast and calibrate for launch
float launch_altimeter = inhg_to_hpa(30.49); // [HPa]
float target_apogee = feet_to_meters(10000.0); // [meters], AGL

// Simulation only
float launch_angle = 0; // [degrees], from vertical
float thrust_modulation = 1; // [multiplier]

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
    -1.1070499962858763e-43,
    2.5806432794016635e-39,
    -2.636210060201722e-35,
    1.5132524137823342e-31,
    -5.0112748823258835e-28,
    6.95427948562038e-25,
    1.720364479867944e-21,
    -1.2246435282033922e-17,
    3.5264369266685235e-14,
    -6.47694588341369e-11,
    8.261143395929143e-08,
    -7.443880627462895e-05,
    0.046660626710584845,
    -19.392912842041383,
    4806.724131608678,
    -537381.7572829655,
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

  // Modulate thrust to simulate performance deviation in reality
  thrust = thrust * thrust_modulation;

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
  if (altitude < 0.002) return 9.792111053673725;
  if (altitude > 5155.169) return 9.776218842883042;

  // These constants are obtained from ../Utilities/gravity_extractor.py using the OpenRocket simulation
  double consts[] = {
    -3.154264237616718e-21,   // P1
    3.078897958768757e-17,   // P2
    -7.790107133693638e-14,   // P3
    -3.8386869819244304e-11,   // P4
    -2.805004301604471e-06,   // P5
    9.792111059283734,   // P6
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

  if (mach < 0.101) {
    // Plateau drag curve at low speed on launch
    if (height < 150) return 0.5199427718028657;

    // Apogee regime:

    // Lower bound
    if (mach < 0.015) return 0.6342688318590177;

    // These constants are obtained from ../Utilities/drag_curve.py using the OpenRocket simulation
    double consts[] = {
      2.976154010485763e+21,    // P1
      -2.534189908828422e+21,    // P2
      9.877838221515783e+20,    // P3
      -2.3353049157933597e+20,    // P4
      3.740567546682588e+19,    // P5
      -4.294394621632756e+18,    // P6
      3.6458490959624083e+17,    // P7
      -2.3276609481967676e+16,    // P8
      1125221220802627.8,    // P9
      -41130541092701.6,    // P10
      1126053343781.6812,    // P11
      -22646101417.727882,    // P12
      323460418.6356909,    // P13
      -3094228.032334539,    // P14
      17702.196587233087,    // P15
      -44.94538426800864,    // P16
    };

    int poly_order = 15;

    // pain
    for (int i = 0; i < poly_order + 1; i++) {
      result = result + ((double) pow(mach, poly_order - i) * consts[i]);
    }
  }

  // Polynomial upper bounds
  if (mach > 1.195) return 0.700364550575614;

  // Subsonic regime:
  if (mach > 0.1 && mach < 0.9) {
    // These constants are obtained from ../Utilities/drag_curve.py using the OpenRocket simulation
    double consts[] = {
      -765896.3093507506,    // P1
      5771470.938647585,    // P2
      -19822242.621078823,    // P3
      41094866.77729983,    // P4
      -57414324.17904674,    // P5
      57156007.22941694,    // P6
      -41802679.40388279,    // P7
      22826976.117353175,    // P8
      -9363992.612196557,    // P9
      2879638.728590026,    // P10
      -657088.902049036,    // P11
      109040.84535222364,    // P12
      -12714.942843019784,    // P13
      982.1595443421627,    // P14
      -44.859904633784645,    // P15
      1.4312088662195603,    // P16
    };

    int poly_order = 15;

    // pain
    for (int i = 0; i < poly_order + 1; i++) {
      result = result + ((double) pow(mach, poly_order - i) * consts[i]);
    }
  }

  // Transonic regime:
  if (mach > 0.9) {
    double consts[] = {
      -4212299.423961634,    // P1
      26596664.36097106,    // P2
      -58245981.30264048,    // P3
      32866275.46387014,    // P4
      50272901.33583769,    // P5
      -40261311.34963774,    // P6
      -65942394.21483676,    // P7
      33585676.278988615,    // P8
      91822538.79022847,    // P9
      -20546887.72375796,    // P10
      -121765757.90060957,    // P11
      29401038.569401,    // P12
      155074584.83816013,    // P13
      -167758356.77176008,    // P14
      70082127.70299605,    // P15
      -10968817.925096473,    // P16
    };

    int poly_order = 15;

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
  if (time > 4505) return 0;

  // These constants are obtained from ../Utilities/thrust_extractor.py using the OpenRocket simulation
  // TODO: piecewise this because it is just way too mf wiggly
  double consts[] = {
    4.940923564843739e-98,
    -1.1230646287392833e-93,
    8.488539419610813e-90,
    -1.1329464586849078e-86,
    -1.2681677749129396e-82,
    1.441429249301959e-79,
    2.482731453263731e-75,
    1.5444419833545796e-72,
    -4.5200980323593215e-68,
    -1.291768466038105e-64,
    6.391839613548839e-61,
    4.084210662653841e-57,
    -5.533707578195202e-54,
    -9.894463095226028e-50,
    -1.2836701454318874e-47,
    2.2420606240466227e-42,
    8.626623065655828e-40,
    -5.356073212430741e-35,
    6.661604036491461e-32,
    1.1166429743664362e-27,
    -7.028957711578119e-24,
    2.1875184067063647e-20,
    -4.346335411895379e-17,
    5.905316525135181e-14,
    -5.587763934942331e-11,
    3.656550778437937e-08,
    -1.6109874883422017e-05,
    0.004548399710292233,
    -0.7571652836749488,
    64.58681375783425,
    67.15725493723727,
  };

  int poly_order = 30;
  double result = 0;

  // pain
  for (int i = 0; i < poly_order + 1; i++) {
    result = result + ((double) pow(time, poly_order - i) * consts[i]);
  }

  return (float) result;
}

float get_mass(float time) { // [ms]
  // Range of polynomial validity
  if (time < 0) return 28.0;
  if (time > 10000) return 23.1;

  // These constants are obtained from ../Utilities/mass_extractor.py using the OpenRocket simulation
  double consts[] = {
    1.0788572078920905e-59,
    -1.965044850507821e-54,
    1.6019147786992172e-49,
    -7.701114535971454e-45,
    2.4197626404489194e-40,
    -5.197506707505949e-36,
    7.735033338830999e-32,
    -7.8676029378262415e-28,
    5.179876968374883e-24,
    -1.8602625468308982e-20,
    6.4692254674482195e-18,
    2.2299763904723932e-13,
    -7.54346013896376e-10,
    7.843078531606863e-07,
    -0.0014037248273447798,
    28.052539304019465,
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
