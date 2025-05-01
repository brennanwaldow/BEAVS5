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
#include "InterpolationLib.h"


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
    // SIM -- False telemetry data from On-Board Simulation
    // FIELD -- Live telemetry data from flight instruments
enum { STOWED, ZEROING, MAX_BRAKING, ACTIVE };
    // STOWED -- Flight computer runs, BEAVS blades remain flush with Outer Diameter
    // ZEROING -- For blade installation / integration, blade gear returns to zero position at Inner Diameter
    // ACTIVE -- PID loop controls blade deflection

// TODO: SET TO FIELD/ACTIVE BEFORE FLIGHT
int BEAVS_mode = SIM;
int BEAVS_control = ACTIVE;

enum { SEA_LEVEL = 0, BROTHERS_OR = 1380 };
float launch_altitude = BROTHERS_OR; // [meters]
// TODO: Obtain pressure forecast and calibrate for launch
float launch_altimeter = inhg_to_hpa(30.49); // [HPa]
float target_apogee = feet_to_meters(10000.0); // [meters], AGL

// Simulation only
float launch_angle = 0; // [degrees], from vertical
float thrust_modulation = 1; // [multiplier]; modulate thrust curve to test performance under deviation from theoretical
float blade_modulation = 1; // [multiplier]; modulate blade drag coefficient to test performance under deviation from theoretical

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

double kp = 5.600e-05 * 3;
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

enum { PREFLIGHT, ARMED, FLIGHT, COAST, OVERSHOOT, DESCENT };
    // PREFLIGHT -- Rocket is on the ground, safed with Remove Before Flight pin installed, blades flush with Inner Diameter
    // ARMED -- Remove Before Flight pin removed, startup animation played, blades flush with Outer Diameter
    // FLIGHT -- Motor ignition detected, currently burning and accelerating
    // COAST -- Motor burnout detected, BEAVS blades running according to BEAVS Control enum
    // OVERSHOOT -- Target apogee passed, last-ditch full extension deployment of blades to minimize overshoot
    // DESCENT -- Apogee detected, blades retracted flush with Outer Diameter and control loop ceased
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

  if (BEAVS_control == ACTIVE) {
    tick_PID();

    command_deflection(u);
  }

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
    if (BEAVS_control == ZEROING) {
      command_deflection(0);
    } else if (BEAVS_control == MAX_BRAKING) {
      command_deflection(1);
    } else {
      command_deflection(1);
      delay(1500);
      command_deflection(0);
      delay(1500);
      command_deflection(0.5);
      delay(1000);
      command_deflection(0);
      delay(1000);
    }
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
void tick_PID() {
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

  float speed_of_sound = (-0.0039389935746399 * altitude) + 340.3888956290942;
  float Mach = abs(velocity) / speed_of_sound;
  float Cd_rocket = get_Cd(Mach);
  float air_density = (-6.159058310425128e-14 * pow(altitude, 3) + 4.23898628189357e-09 * pow(altitude, 2) + (-0.00011741752091555341 * altitude) + 1.2251271371466361);

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
  float A_ref = 0.017424685;
  float virtual_deflection = max((virtual_angle - 8.86) / (150 - 8.86), 0);
  float A_beavs = ((feet_to_meters(1.632 / 12) * feet_to_meters(2.490 / 12)) * 2) * virtual_deflection;
  // TODO: Polyfit from Ansys Fluent god help us
  float Cd_beavs = 4.8 * (sqrt(A_beavs / A_ref)) * blade_modulation;
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
  // if (launch_clock < (4.505) * 1000) {
  //   altitude = 796.68;
  //   acceleration = 0;
  //   velocity = 285.398;
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
  Serial.print(get_Fd_BEAVS(velocity, 4.8 * (sqrt(((feet_to_meters(1.632 / 12) * feet_to_meters(2.490 / 12)) * 2) / A_ref)), ((feet_to_meters(1.632 / 12) * feet_to_meters(2.490 / 12)) * 2), air_density));
  Serial.print(" ");
  Serial.print(mass * gravity(altitude));
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

float get_thrust(float time) {
  // Range of validity
  if (time < 0) return 0;
  if (time > 4505) return 0;
  
  double xValues[139] = { 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 170.0, 180.0, 190.0, 200.0, 210.0, 220.0, 230.0,
  240.0, 250.0, 260.0, 270.0, 280.0, 290.0, 300.0, 310.0, 320.0, 330.0, 340.0, 350.0, 360.0, 370.0, 380.0, 390.0, 400.0, 410.0, 420.0, 430.0, 440.0, 450.0, 460.0, 470.0, 480.0, 
  490.0, 500.0, 510.0, 520.0, 530.0, 540.0, 550.0, 560.0, 570.0, 580.0, 595.0, 618.0, 651.0, 701.0, 751.0, 801.0, 851.0, 901.0, 951.0, 1000.9999999999999, 1051.0, 1101.0, 1151.0, 
  1201.0, 1251.0, 1301.0, 1351.0, 1401.0, 1451.0, 1501.0, 1551.0, 1601.0, 1651.0, 1701.0, 1751.0, 1801.0, 1851.0, 1901.0, 1951.0, 2001.0, 2051.0, 2101.0, 2151.0, 2201.0, 2251.0, 2301.0, 
  2351.0, 2399.0, 2449.0, 2499.0, 2549.0, 2599.0, 2649.0, 2698.0, 2748.0, 2798.0, 2848.0, 2898.0, 2948.0, 2998.0, 3048.0, 3098.0, 3148.0, 3198.0, 3248.0, 3298.0, 3348.0, 3398.0, 3448.0, 
  3498.0, 3548.0, 3598.0, 3648.0, 3698.0, 3748.0, 3798.0, 3848.0, 3898.0, 3948.0, 3998.0, 4048.0, 4098.0, 4148.0, 4198.0, 4248.0, 4298.0, 4348.0, 4398.0, 4448.0, 4498.0, 4501.0 };
  
  double yValues[139] = { 339.816, 1019.428, 1699.04, 2042.049, 2048.454, 2054.86, 2061.249, 2067.623, 2073.997, 2080.355, 2086.696, 2093.037, 2099.361, 2105.667, 2111.974, 2118.263, 
  2124.534, 2130.804, 2137.056, 2143.29,  2149.523, 2155.737, 2161.931, 2168.126, 2174.3, 2180.454, 2186.608, 2192.741, 2198.853, 2204.965, 2211.055, 2217.124, 2223.193, 2229.239, 2235.262, 
  2241.286, 2247.287, 2253.264, 2259.241, 2265.194, 2271.124, 2277.053, 2282.957, 2288.837, 2294.717, 2300.572, 2306.401, 2312.23, 2318.033, 2323.809, 2329.586, 2335.336, 2341.058, 2346.781, 
  2352.476, 2358.144, 2363.811, 2369.45, 2376.464, 2386.944, 2402.524, 2425.428, 2452.31, 2478.659, 2504.457, 2529.677, 2554.306, 2578.33, 2601.719, 2624.464, 2646.55, 2667.951, 2688.656, 
  2708.653, 2727.918, 2746.44, 2764.21, 2781.202, 2797.412, 2812.829, 2827.43, 2841.212, 2854.167, 2866.272, 2877.527, 2887.926, 2897.449, 2906.095, 2913.863, 2920.733, 2926.708, 2931.788, 
  2935.955, 2939.215, 2941.569, 2942.982, 2943.519, 2943.169, 2941.896, 2939.714, 2936.622, 2932.637, 2927.76, 2921.957, 2915.248, 2907.653, 2899.17, 2889.8, 2879.564, 2868.462, 2856.498, 
  2685.013, 1499.291, 1430.697, 1423.762, 1416.535, 1409.016, 1401.214, 1393.131, 1384.769, 1376.136, 1367.234, 1358.067, 870.73, 630.195, 627.014, 623.744, 620.385, 616.939, 613.406, 609.788, 
  606.084, 602.298, 598.43, 594.48, 590.45, 586.342, 425.427, 9.037 };

  float thrust = Interpolation::Linear(xValues, yValues, 139, time, false);

  return thrust;
}

float get_thrust_old(float time) { // [ms]
  // Range of polynomial validity
  if (time < 0) return 0;
  if (time > 4505) return 0;

  double result = 0;

  if (time < 37) {
    // These constants are obtained from ../Utilities/thrust_extractor.py using the OpenRocket simulation
    double consts[] = {
      -4.418372606013904e-13,   // P1
      -1.1652407369607583e-11,   // P2
      -2.7769362913834166e-10,   // P3
      -5.1226655302972864e-09,   // P4
      -1.2025207751754383e-08,   // P5
      5.745404750093319e-06,   // P6
      0.00042155390307917076,   // P7
      0.021542269846367295,   // P8
      0.8540112017390363,   // P9
      21.638784656324066,   // P10
      11.801775916630444,   // P11
    };

    int poly_order = 10;

    // pain
    for (int i = 0; i < poly_order + 1; i++) {
      result = result + ((double) pow(time, poly_order - i) * consts[i]);
    }
  } else if (time < 3247) {
    // These constants are obtained from ../Utilities/thrust_extractor.py using the OpenRocket simulation
    double consts[] = {
      -4.683266251558473e-60,   // P1
      1.1787016408257099e-55,   // P2
      -1.3053001934264523e-51,   // P3
      8.195452116379321e-48,   // P4
      -3.0473216375816937e-44,   // P5
      5.53373569555978e-41,   // P6
      5.507473272370591e-38,   // P7
      -6.739586893490672e-34,   // P8
      2.2101582922098247e-30,   // P9
      -4.4402065806942574e-27,   // P10
      6.143566143807555e-24,   // P11
      -6.05278350217028e-21,   // P12
      4.284737485017909e-18,   // P13
      -2.194460129948485e-15,   // P14
      8.375731259447085e-13,   // P15
      -2.565166513613027e-10,   // P16
      6.694478794385455e-08,   // P17
      -1.370435689909412e-05,   // P18
      0.0017398110068922645,   // P19
      0.5265652484823354,   // P20
      2019.1472070293255,   // P21
    };

    int poly_order = 20;

    // pain
    for (int i = 0; i < poly_order + 1; i++) {
      result = result + ((double) pow(time, poly_order - i) * consts[i]);
    }
  } else if (time < 3790) {
    // These constants are obtained from ../Utilities/thrust_extractor.py using the OpenRocket simulation
    double consts[] = {
      2.2999274937804016e-25,   // P1
      -5.004400272600633e-21,   // P2
      4.114562056363646e-17,   // P3
      -1.3098847515170642e-13,   // P4
      -1.2894400892907893e-10,   // P5
      1.7959909077218326e-06,   // P6
      -0.0009872286967884773,   // P7
      -22.647280015673868,   // P8
      83561.76803758714,   // P9
      -123864165.02678245,   // P10
      69989246965.8426,   // P11
    };

    int poly_order = 10;

    // pain
    for (int i = 0; i < poly_order + 1; i++) {
      result = result + ((double) pow(time, poly_order - i) * consts[i]);
    }
  } else {
    // These constants are obtained from ../Utilities/thrust_extractor.py using the OpenRocket simulation
    double consts[] = {
      -1.2346768260727624e-60,   // P1
      2.1297977209016264e-56,   // P2
      -8.929946035918273e-53,   // P3
      -2.3373387683005864e-49,   // P4
      1.0923734713282396e-45,   // P5
      6.600229944588467e-42,   // P6
      2.0389200818999012e-39,   // P7
      -1.1089382926201572e-34,   // P8
      -4.794148791597963e-31,   // P9
      6.958813782877615e-29,   // P10
      9.370597421141463e-24,   // P11
      3.8796179911512314e-20,   // P12
      -1.8128133922091875e-17,   // P13
      -8.357144568735814e-13,   // P14
      -2.823117715676497e-09,   // P15
      7.268801386890557e-06,   // P16
      0.07902047087140077,   // P17
      -5.076544891217461,   // P18
      -1792317.8675069313,   // P19
      5199715706.81694,   // P20
      -4498769262459.649,   // P21
    };

    int poly_order = 20;

    // pain
    for (int i = 0; i < poly_order + 1; i++) {
      result = result + ((double) pow(time, poly_order - i) * consts[i]);
    }
  }

  return (float) result;
}

float get_mass(float time) { // [ms]
  // Range of polynomial validity
  if (time < 0) return 28.0;
  if (time > 4505) return 23.1;

  // These constants are obtained from ../Utilities/mass_extractor.py using the OpenRocket simulation
  double consts[] = {
    -6.465450605250409e-51,
    6.486929577503462e-46,
    -1.638556411639733e-41,
    2.0629686928364894e-37,
    -1.5728120757775486e-33,
    7.908562099671616e-30,
    -2.734533899861233e-26,
    6.622394495150212e-23,
    -1.1262294202635161e-19,
    1.3307462216006634e-16,
    -1.0661505179695941e-13,
    5.554925035194835e-11,
    -1.7565570557915666e-08,
    2.830531428120959e-06,
    -0.001221279756568574,
    28.01877845821919,
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
