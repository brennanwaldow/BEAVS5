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
#include <SdFat.h>
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
int BEAVS_mode = FIELD;
int BEAVS_control = ACTIVE;

enum { SEA_LEVEL = 0, TESTING = 67, BROTHERS_OR = 1380 };
float launch_altitude = BROTHERS_OR; // [meters]
// TODO: Obtain pressure forecast and calibrate for launch
float launch_altimeter = inhg_to_hpa(30.12); // [HPa]
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
bool BMP_failure = true;

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool BNO_failure = true;

// Servo
int servo_pin = 28; // GPIO 28 / Physical Pin 34

// SD
SdFs SD;
FsFile log_file;
FsFile telemetry_file;
int log_index;
String log_filename;
String telemetry_filename;
bool SD_failure = true;

const int SD_pin_MISO = 16;
const int SD_pin_MOSI = 19;
const int SD_pin_CS = 17;
const int SD_pin_SCK = 18;

#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CONFIG SdSpiConfig(SD_pin_CS, DEDICATED_SPI, SPI_CLOCK)

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

double kp = 5.600e-05 * 2;
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

double roll_angle = 0;             // [deg]
double relative_roll_angle = 0;    // [deg]
double pitch_angle = 0;            // [deg]
double pitch_angle_x = 0;          // [deg]
double pitch_angle_y = 0;          // [deg]

float drag_force_approx = 0;       // [N]
float drag_force_expected = 0;     // [N]


long clock_time = 0;               // [ms]
long curr_time = 0;                // [micro s]
long pid_clock_time = 0;           // [ms]
long launch_timestamp = 0;         // [ms]
long apogee_timestamp = 0;         // [ms]
bool log_terminated = false;       // [bool]

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

// Core 1
void setup() {
  Serial.begin(115200);
  // Wait for serial to initialize
  delay(3000);

  Serial.println("Starting up hardware...");

  Wire.begin();

  if (!bmp.begin_I2C(0x77)) {
    Serial.println("ERROR: BMP390 failed to initialize.");
  } else {
    BMP_failure = false;
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 failed to initialize.");
  } else {
    BNO_failure = false;
  }
  bno.setExtCrystalUse(true);

  pinMode(servo_pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize SD card
  bool setRX(SD_pin_MISO);
  bool setTX(SD_pin_MOSI);
  bool setSCK(SD_pin_SCK);
  bool sdInitialized = false;
  sdInitialized = SD.begin(SD_CONFIG);

  if (!sdInitialized) {
    Serial.println("ERROR: SD card writer failed to initialize.");
  } else {
    SD_failure = false;

    // Search for the first available log slot
    for (int i = 1; i < 10000; i++) {
      String filename = "Logs/log_" + String(i) + ".txt";
      if (SD.exists(filename) == 1) continue;
      log_index = i;
      log_filename = filename;
      telemetry_filename = "Data/data_" + String(log_index) + ".csv";

      log("Initiating logger with index " + String(log_index) + ".");

      write_telemetry_headers();
      break;
    }

    log("Startup complete.");
  }

  // Ensure servo is fully stowed
  command_deflection(-1);
}

// Core 2
void setup1() {

}

// -----   Control Loop   -----

// Core 1
void loop() {
  curr_time = micros();

  if ((SD_failure == true) || (BMP_failure == true) || (BNO_failure == true)) {
    int error_strobe = round(((curr_time) % 1000000) / 1000000.0);
    digitalWrite(LED_BUILTIN, error_strobe);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  switch(flight_phase) {
    case PREFLIGHT:
      preflight_loop(1);
      break;
    case ARMED:
      ready_loop(1);
      break;
    case FLIGHT:
      flight_loop(1);
      break;
    case OVERSHOOT:
      overshoot_loop(1);
      break;
    case COAST:
      coast_loop(1);
      break;
    case DESCENT:
      descend_loop(1);
      break;
  }

  clock_time = curr_time;

  delay(5);
}

// Core 2
void loop1() {
  switch(flight_phase) {
    case PREFLIGHT:
      preflight_loop(2);
      break;
    case ARMED:
      ready_loop(2);
      break;
    case FLIGHT:
      flight_loop(2);
      break;
    case OVERSHOOT:
      overshoot_loop(2);
      break;
    case COAST:
      coast_loop(2);
      break;
    case DESCENT:
      descend_loop(2);
      break;
  }

  delay(5);
}


void preflight_loop(int core) {
  if (core == 1) {
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(500);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(1000);

    if (millis() > 3000) arm();

    collect_telemetry();
    calculate_telemetry();
    // TODO: disable telemetry write on ground for final flight
    write_telemetry();
  } else if (core == 2) {

  }
}

void ready_loop(int core) {
  if (core == 1) {
    // digitalWrite(LED_BUILTIN, HIGH);

    // delay(100);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(100);

    collect_telemetry();
    calculate_telemetry();
    // TODO: disable telemetry write on ground for final flight
    write_telemetry();

    if (acceleration > 10) launch();
  } else if (core == 2) {

  }
}

void flight_loop(int core) {
  if (core == 1) {
    collect_telemetry();
    calculate_telemetry();
    write_telemetry();

    if (acceleration < 5) coast();
  } else if (core == 2) {

  }
}

void coast_loop(int core) {
  if (core == 1) {
    collect_telemetry();
    calculate_telemetry();
    write_telemetry();

    if (BEAVS_control == ACTIVE) {
      tick_PID();

      command_deflection(u);
    }

    if (height > target_apogee) overshoot();

    // TODO the altitude reading is TOO NOISY for this cutoff: will have to add additional conditions
    if (max_height > height && (millis() - max_height_clock) > 500) {
      descend();
    }
  } else if (core == 2) {

  }
}

void overshoot_loop(int core) {
  if (core == 1) {
    collect_telemetry();
    calculate_telemetry();
    write_telemetry();

    if (max_height > (height + 50)) descend();

    if (max_height > height && (millis() - max_height_clock) > 500) {
      descend();
    }
  } else if (core == 2) {

  }
}

void descend_loop(int core) {
  if (core == 1) {
    // Continue logging for five minutes after apogee
    if ((millis() - apogee_timestamp) < 300000) {
      collect_telemetry();
      calculate_telemetry();
      write_telemetry();
    } else if (log_terminated == false) {
      log("Five minutes after apogee. Logging terminated.");
      log_terminated = true;
    }
  } else if (core == 2) {

  }
}



// -----   Phase Changeovers   -----
void arm() {
  // SAFETY PIN REMOVED: Arm BEAVS monitoring and initiate startup
  flight_phase = ARMED;

  log("Safety pin removed. BEAVS arming.");

  if (BEAVS_control == STOWED) {
    log("BEAVS control: STOWED.");
    command_deflection(0);
  } else if (BEAVS_mode == FIELD) {
    command_deflection(1);
    delay(2000);
    command_deflection(0.5);
    delay(1000);
    command_deflection(0);
  } else if (BEAVS_mode == SIM) {
    log("BEAVS running in SIM mode.");
    if (BEAVS_control == ZEROING) {
      command_deflection(0);
    } else if (BEAVS_control == MAX_BRAKING) {
      command_deflection(1);
    } else {
      command_deflection(0.3);
      delay(500);
      command_deflection(0);
      delay(500);
      command_deflection(0.3);
      delay(500);
      command_deflection(0);
      delay(500);
      command_deflection(0.3);
      delay(500);
      command_deflection(0);
      delay(500);
      command_deflection(0.3);
      delay(500);
      command_deflection(0);
      delay(500);
    }
  }

  // Recalibrate launch ground level to current altitude?
}

void disarm() {
  // SAFETY PIN REINSERTED: Return to Disarmed state
  flight_phase = PREFLIGHT;
  
  log("Safety pin reinserted. Disarming.");
}

void launch() {
  // ENGINE IGNITION: Arm BEAVS monitoring for motor cutoff
  flight_phase = FLIGHT;
  launch_timestamp = millis();

  log("Motor ignition detected.");
}

void coast() {
  // ENGINE CUTOFF: Deploy BEAVS
  flight_phase = COAST;
  
  log("Coast phase entered, beginning deployment.");
}

void overshoot() {
  // APOGEE OVERSHOT: Last full extension regime on airbrake to minimize further overshoot
  flight_phase = OVERSHOOT;
  if (BEAVS_control == ACTIVE) command_deflection(1);

  log("Target apogee overshot! Extending full braking.");
}

void descend() {
  // APOGEE REACHED: BEAVS safing, PID shutdown
  flight_phase = DESCENT;
  command_deflection(0);

  log("Apogee reached.");
  log("Altitude achieved: " + String(height, 2) + " m AGL   //   " + String(meters_to_feet(height), 2) + " ft AGL");

  apogee_timestamp = millis();
}



// -----   Functions   -----

void log(String message) {
  log_file = SD.open(log_filename, O_CREAT | O_WRITE | O_APPEND);
  String timestamp = "[" + String(millis() / 1000.0, 2) + "s] ";
  log_file.println(timestamp + message);
  log_file.close();
  Serial.println(timestamp + message);
}

void write_telemetry() {
  telemetry_file = SD.open(telemetry_filename, O_CREAT | O_WRITE | O_APPEND);
  String timestamp = String(millis() / 1000.0, 4);
  String telemetry_string = timestamp + ","
                          + altitude + ","
                          + height + ","
                          + velocity + ","
                          + acceleration + ","
                          + commanded_angle + ","
                          + flight_phase + ","
                          + drag_force_approx + ","
                          + drag_force_expected + ","
                          + target_velocity + ","
                          + error1 + ","
                          + roll_angle + ","
                          + relative_roll_angle + ","
                          + pitch_angle_x + ","
                          + pitch_angle_y + ","
                          + pitch_angle;
  telemetry_file.println(telemetry_string);
  telemetry_file.close();
}

void write_telemetry_headers() {
  telemetry_file = SD.open(telemetry_filename, O_CREAT | O_WRITE | O_APPEND);
  String timestamp = String(millis() / 1000.0, 4);
  String telemetry_string = String("# Time [s],")
                          + String("# Altitude [m],")
                          + String("# Height [m AGL],")
                          + String("# Velocity [m/s],")
                          + String("# Acceleration [m/s^2],")
                          + String("# Commanded Angle [deg],")
                          + String("# Flight Phase [int enum],")
                          + String("# Approximate Drag Force [N],")
                          + String("# Expected Drag Force [N],")
                          + String("# Target Velocity [m/s],")
                          + String("# Velocity Error [m/s],")
                          + String("# Roll [deg],")
                          + String("# Relative Roll [deg],")
                          + String("# Pitch (x) [deg],")
                          + String("# Pitch (y) [deg],")
                          + String("# Pitch (from vertical) [deg]");
  telemetry_file.println(telemetry_string);
  telemetry_file.close();
}

void collect_telemetry() {
  if (BEAVS_mode == SIM) get_trolled_idiot();
  else {
    // BMP390
    if (!bmp.performReading()) {
      Serial.println("BMP390 Reading failed!");
      BMP_failure = true;
    } else {
      BMP_failure = false;
    }

    float prev_altitude = altitude;
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    // Serial.println(altitude);

    double dt = (micros() - clock_time) / (double) 1000000;
    velocity = (altitude - prev_altitude) / dt;
    clock_time = micros();

    // BNO055
    sensors_event_t accelerometer, gyroscope;
    bno.getEvent(&accelerometer, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&gyroscope, Adafruit_BNO055::VECTOR_EULER);

    // TODO: 3-axis -> magnitude?
    sensors_event_t* accelerometer_event = &accelerometer;
    accelerometer_event->type == SENSOR_TYPE_LINEAR_ACCELERATION;
    acceleration = accelerometer_event->acceleration.z;

    sensors_event_t* gyro_event = &gyroscope;
    gyro_event->type == SENSOR_TYPE_ORIENTATION;

    float new_roll = gyro_event->orientation.x;

    if (new_roll < 20 && roll_angle > 340) {
      relative_roll_angle = relative_roll_angle + (360 - roll_angle) + new_roll;
    } else if (new_roll > 340 && roll_angle < 20) {
      relative_roll_angle = relative_roll_angle - roll_angle - (360 - new_roll);
    } else {
      relative_roll_angle = relative_roll_angle + (new_roll - roll_angle);
    }

    roll_angle = new_roll;

    pitch_angle_x = gyro_event->orientation.y;
    pitch_angle_y = gyro_event->orientation.z;

    Serial.print(roll_angle);
    Serial.print(" ");
    Serial.print(relative_roll_angle);
    Serial.print(" ");
    Serial.print(pitch_angle_x);
    Serial.print(" ");
    Serial.print(pitch_angle_y);
    Serial.print(" ");
    Serial.print(pitch_angle);
    Serial.print(" ");

    Serial.print(altitude);
    Serial.print(" ");
    Serial.print(prev_altitude);
    Serial.print(" ");
    Serial.print(dt);
    Serial.print(" ");
    Serial.print(acceleration);
    Serial.print(" ");
    Serial.println(velocity);
    // TODO: Break delay into main loop clock!
    delay(20);
  }
}

void calculate_telemetry() {
  height = altitude - launch_altitude;
  if (height > max_height) {
    max_height = height;
    max_height_clock = millis();
  }

  // Calculate approximate drag force experienced
  float mass_approx = get_mass(millis() - launch_timestamp);
  float drag_acceleration_approx = acceleration - (-gravity(altitude));
  drag_force_approx = -(drag_acceleration_approx * mass_approx);

  // Calculate expected drag force due to BEAVS

  // TODO: Merge this with get_beavs_drag function or something idk
  float A_ref = 0.02043171233;
  float virtual_deflection = max((virtual_angle - 8.86) / (150 - 8.86), 0);
  float A_beavs = ((feet_to_meters(1.632 / 12) * feet_to_meters(2.490 / 12)) * 2) * virtual_deflection;
  
  float speed_of_sound = (-0.003938999995558203 * altitude) + 340.3888999387908;
  float Mach = abs(velocity) / speed_of_sound;
  float Cd_rocket = get_Cd(Mach);
  float air_density = (-6.866808372788853e-14 * pow(altitude, 3) + 4.309128823975302e-09 * pow(altitude, 2) + (-0.00011761140418837493 * altitude) + 1.2252514803412429);

  float Cd_beavs = 4.8 * (sqrt(A_beavs / A_ref)) * blade_modulation;
  float Cd = Cd_rocket + (Cd_beavs * (A_beavs / A_ref));

  drag_force_expected = abs(0.5 * air_density * (velocity * velocity) * Cd * A_ref);

  // Convert two-axis pitch into single-axis pitch
  float a1 = degrees_to_radians(pitch_angle_x);
  float a2 = degrees_to_radians(pitch_angle_y);

  float h1 = cos(a1);
  float l1 = sin(a1);

  float l2 = h1 * sin(a2);
  float h = h1 * cos(a2);

  float l = sqrt((l1 * l1) + (l2 * l2));

  // TODO EMERGENCY: do something to make sure i dont divide by zero during flight lmfao
  pitch_angle = abs(radians_to_degrees(atan(l / h)));

  if ((abs(pitch_angle_x) > 90) || (abs(pitch_angle_y) > 90)) {
    pitch_angle = 180 - pitch_angle;
  }
}

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

  // if (error1 < 0) u = 0;
  
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
// ONLY VALID for launch site: BROTHERS
float velocity_lookup() {
  // Range of polynomial validity: Burnout to apogee
  if (height < 776.2875822000002) return 283.67155136563815;
  if (height > target_apogee) return 0;

  // These constants are obtained from ../Utilities/lookup_table.py using the OpenRocket simulation
  double consts[] = {
    -1.340426210934022e-43,
    3.1554665860233744e-39,
    -3.2600537121293397e-35,
    1.8975851608905006e-31,
    -6.413542632311314e-28,
    9.43002155389309e-25,
    1.9939331053310826e-21,
    -1.5355461150548986e-17,
    4.5346069870725215e-14,
    -8.490920972495264e-11,
    1.1027739467619363e-07,
    -0.00010119218778210783,
    0.06463422589117263,
    -27.396439706609893,
    6932.349887135,
    -792243.509439591,
  };

  int poly_order = 15;
  double result = 0;

  // pain
  for (int i = 0; i < poly_order + 1; i++) {
    result = result + ((double) pow(height, poly_order - i) * consts[i]);
  }

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

  long launch_clock = millis() - 15000;

  float speed_of_sound = (-0.003938999995558203 * altitude) + 340.3888999387908;
  float Mach = abs(velocity) / speed_of_sound;
  float Cd_rocket = get_Cd(Mach);
  float air_density = (-6.866808372788853e-14 * pow(altitude, 3) + 4.309128823975302e-09 * pow(altitude, 2) + (-0.00011761140418837493 * altitude) + 1.2252514803412429);

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
  float A_ref = 0.02043171233;
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
  Serial.print(drag_force_approx);
  Serial.print(" ");
  Serial.print(drag_force_expected);
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
  if (altitude < 1380.00067880291) return 9.800601153885829;
  if (altitude > 6575.6628767) return 9.784637022898936;

  // These constants are obtained from ../Utilities/gravity_extractor.py using the OpenRocket simulation
  double consts[] = {
    1.9820614413562718e-22,   // P1
    -5.3066518070855185e-18,   // P2
    5.398468733344706e-14,   // P3
    -2.610086439273407e-10,   // P4
    -2.4859764599374713e-06,   // P5
    9.804405246460368,   // P6
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
  // Range of polynomial validity
  if (mach > 1.1967322) return 0.6999506;

  if (mach < 0.1000412) {
    // Plateau drag curve at low speed on launch
    if (height < 150) return 0.5198125;

    if (mach < 0.0218646) return 0.6121415;
  }

  double machValues[435] = { 0.1000412, 0.1015506, 0.1030994, 0.1046778, 0.1062021, 0.1077064, 0.1092126, 0.1107356, 0.1122538, 0.1137099, 0.1152098, 0.1167975, 0.1183759, 0.1199052, 0.1214738, 0.1230561, 0.1245801, 0.1260949,
  0.1276424, 0.1291635, 0.1307118, 0.1323242, 0.1339504, 0.1355597, 0.1371361, 0.1386867, 0.1402017, 0.1417595, 0.1433569, 0.1449348, 0.1464978, 0.1480768, 0.1496509, 0.1512092, 0.1527589, 0.1543301, 0.1558746, 0.1574346,
  0.1590458, 0.160643, 0.1622213, 0.1638021, 0.1654129, 0.1670202, 0.1686087, 0.1701813, 0.1717713, 0.1733559, 0.1749363, 0.1764932, 0.1780525, 0.1796486, 0.1812048, 0.1827711, 0.1843824, 0.1860053, 0.1876253, 0.1892065,
  0.1908041, 0.1924316, 0.1940188, 0.1955794, 0.1971677, 0.198779, 0.2004054, 0.20202, 0.203601, 0.2051951, 0.2068294, 0.208447, 0.2100727, 0.2117289, 0.2133704, 0.2150038, 0.216622, 0.218237, 0.2198546, 0.2214548, 0.2230793,
  0.2247169, 0.2263494, 0.2279754, 0.229588, 0.2312127, 0.2328523, 0.2344876, 0.2361355, 0.2377945, 0.2394251, 0.2410743, 0.2427447, 0.2443802, 0.2460114, 0.247656, 0.2493168, 0.2509978, 0.2526577, 0.2542786, 0.2559111,
  0.2575764, 0.2592308, 0.2608655, 0.2625203, 0.2641863, 0.2658437, 0.2675107, 0.269182, 0.2708357, 0.2724912, 0.2741635, 0.2758558, 0.2775358, 0.2792181, 0.2808928, 0.2825641, 0.2842713, 0.2859705, 0.2876732, 0.2893839,
  0.2911038, 0.2927911, 0.2944531, 0.2961413, 0.2978252, 0.2995087, 0.3012432, 0.3029729, 0.304677, 0.3063746, 0.3080665, 0.3097864, 0.3115048, 0.3132269, 0.3149418, 0.3166467, 0.3183725, 0.3200873, 0.3218007, 0.3235279,
  0.3252596, 0.3269858, 0.3287191, 0.3304615, 0.3322003, 0.3339396, 0.3356714, 0.3373953, 0.3391401, 0.3408719, 0.3426149, 0.3443917, 0.3461653, 0.347937, 0.3496706, 0.3514142, 0.3532066, 0.3549926, 0.3567432, 0.3585166,
  0.3603137, 0.3621042, 0.3638851, 0.3656767, 0.3674721, 0.369266, 0.3710653, 0.3728522, 0.3746561, 0.3764602, 0.3782648, 0.3800852, 0.3818809, 0.3836754, 0.3854971, 0.3873279, 0.3891537, 0.3909547, 0.3927619, 0.3945892,
  0.3964147, 0.3982315, 0.4000658, 0.4019182, 0.4037601, 0.4056005, 0.4074494, 0.4092926, 0.4111406, 0.4130084, 0.4148865, 0.4167594, 0.4186193, 0.4204791, 0.4223548, 0.424245, 0.426129, 0.4280112, 0.429915, 0.4318193,
  0.4337186, 0.4356104, 0.4374974, 0.4394087, 0.4413286, 0.4432464, 0.4451866, 0.4471392, 0.4490646, 0.4509881, 0.4529276, 0.4548748, 0.4568184, 0.4587533, 0.4607026, 0.4626597, 0.4646033, 0.4665617, 0.4685328, 0.470487,
  0.4724588, 0.4744556, 0.4764518, 0.478444, 0.4804306, 0.482419, 0.4844287, 0.4864526, 0.488465, 0.4904745, 0.4924905, 0.4945127, 0.496543, 0.4985849, 0.5006246, 0.5026684, 0.5047296, 0.5067988, 0.5088494, 0.5109087,
  0.5129949, 0.5150661, 0.5171371, 0.5192164, 0.5213051, 0.5234118, 0.5255249, 0.5276376, 0.5297599, 0.5318772, 0.5339944, 0.536122, 0.538247, 0.5403887, 0.5425356, 0.5446915, 0.546855, 0.5490241, 0.5512057, 0.5533923,
  0.555577, 0.5577626, 0.5599552, 0.5621477, 0.5643487, 0.5665572, 0.5687822, 0.5710126, 0.5732457, 0.5754948, 0.5777548, 0.5800237, 0.5823074, 0.5845916, 0.5868689, 0.5891487, 0.5914366, 0.593743, 0.596069, 0.5984042,
  0.6007429, 0.6030865, 0.6054516, 0.607815, 0.6101703, 0.6125393, 0.6149162, 0.6173105, 0.6197054, 0.6221088, 0.624537, 0.6269741, 0.6294076, 0.6318479, 0.6342952, 0.636755, 0.6392392, 0.6417244, 0.644208, 0.6466995,
  0.6492143, 0.6517477, 0.6542851, 0.6568338, 0.6593965, 0.6619614, 0.6645365, 0.6671157, 0.669698, 0.6723097, 0.674924, 0.6775498, 0.6802014, 0.6828528, 0.6855045, 0.6881831, 0.6908762, 0.6935744, 0.6962901, 0.6990175,
  0.7017633, 0.7045172, 0.7072777, 0.7100622, 0.7128657, 0.7156737, 0.7184903, 0.7213275, 0.7241786, 0.727056, 0.7299506, 0.7328583, 0.7357842, 0.7387235, 0.741693, 0.7446709, 0.7476464, 0.7506433, 0.7536595, 0.7566894,
  0.7597391, 0.7628085, 0.7659019, 0.7690115, 0.7721404, 0.7752855, 0.7784427, 0.7816297, 0.7848477, 0.7880812, 0.7913401, 0.794615, 0.7979075, 0.8012399, 0.8045906, 0.8079564, 0.8113392, 0.814741, 0.8181742, 0.8216296,
  0.825118, 0.8286428, 0.8321892, 0.8357705, 0.8393751, 0.8430026, 0.8466691, 0.8503708, 0.8541031, 0.8578645, 0.8616548, 0.8654788, 0.8693405, 0.873252, 0.8772006, 0.881174, 0.8852001, 0.8892821, 0.8934026, 0.8975617,
  0.9017737, 0.9060173, 0.9102852, 0.914586, 0.9189159, 0.9232627, 0.92765, 0.9320701, 0.9365125, 0.9409864, 0.9455027, 0.9500556, 0.9546289, 0.9592311, 0.963873, 0.9685517, 0.9732574, 0.9779956, 0.9827668, 0.9875746,
  0.9924245, 0.9973063, 1.0022215, 1.007208, 1.0122505, 1.017363, 1.0225594, 1.0278094, 1.0331233, 1.0385163, 1.0439737, 1.0494877, 1.0550565, 1.0606936, 1.0664029, 1.0721672, 1.0779822, 1.0838467, 1.0897665, 1.0957317,
  1.1017393, 1.1077931, 1.1138937, 1.1200409, 1.1262259, 1.1324605, 1.1387459, 1.1450635, 1.1514104, 1.1577919, 1.1642062, 1.1706441, 1.1771241, 1.1836427, 1.190174, 1.1967322, };

  double cdValues[435] = { 0.5198125, 0.5198629, 0.5199153, 0.5199696, 0.5200228, 0.520076, 0.52013, 0.5201853, 0.5202412, 0.5202956, 0.5203523, 0.5204131, 0.5204743, 0.5205345, 0.5205969, 0.5206608, 0.520723,
  0.5207856, 0.5208504, 0.5209148, 0.5209811, 0.521051, 0.5211224, 0.5211939, 0.5212647, 0.5213352, 0.5214049, 0.5214773, 0.5215523, 0.5216273, 0.5217024, 0.521779, 0.5218563, 0.5219335, 0.5220112, 0.5220907, 0.5221697, 0.5222503,
  0.5223343, 0.5224185, 0.5225026, 0.5225876, 0.522675, 0.5227632, 0.5228511, 0.522939, 0.5230288, 0.523119, 0.5232099, 0.5233002, 0.5233915, 0.5234858, 0.5235785, 0.5236727, 0.5237705, 0.5238698, 0.5239698, 0.5240684,
  0.5241687, 0.5242719, 0.5243734, 0.524474, 0.5245772, 0.5246829, 0.5247904, 0.524898, 0.5250042, 0.5251122, 0.5252238, 0.5253352, 0.525448, 0.5255639, 0.5256797, 0.5257959, 0.5259118, 0.5260285, 0.5261462, 0.5262636,
  0.5263836, 0.5265056, 0.5266281, 0.526751, 0.5268738, 0.5269985, 0.5271252, 0.5272526, 0.5273819, 0.527513, 0.5276428, 0.527775, 0.5279099, 0.528043, 0.5281766, 0.5283123, 0.5284503, 0.528591, 0.5287309, 0.5288685,
  0.5290079, 0.5291512, 0.5292945, 0.5294371, 0.5295824, 0.5297297, 0.5298772, 0.5300265, 0.5301773, 0.5303275, 0.5304788, 0.5306326, 0.5307894, 0.530946, 0.5311039, 0.531262, 0.5314209, 0.5315843, 0.5317479, 0.531913,
  0.5320799, 0.5322488, 0.5324155, 0.5325808, 0.5327497, 0.5329193, 0.5330899, 0.5332667, 0.5334442, 0.5336202, 0.5337965, 0.5339734, 0.5341543, 0.5343361, 0.5345195, 0.5347032, 0.5348869, 0.535074, 0.5352611, 0.5354491,
  0.5356398, 0.5358321, 0.536025, 0.5362198, 0.5364168, 0.5366145, 0.5368136, 0.5370129, 0.5372124, 0.5374156, 0.5376184, 0.5378238, 0.5380343, 0.5382457, 0.5384582, 0.5386672, 0.5388787, 0.5390974, 0.5393165, 0.5395326,
  0.5397527, 0.5399771, 0.5402019, 0.5404268, 0.5406544, 0.5408838, 0.5411142, 0.5413468, 0.541579, 0.5418147, 0.5420518, 0.5422904, 0.5425324, 0.5427725, 0.5430137, 0.54326, 0.5435089, 0.5437586, 0.5440062, 0.5442561,
  0.5445102, 0.5447654, 0.5450209, 0.5452802, 0.5455436, 0.5458069, 0.5460715, 0.5463388, 0.5466068, 0.546877, 0.5471516, 0.5474292, 0.5477076, 0.5479857, 0.5482653, 0.5485488, 0.5488361, 0.5491241, 0.5494134, 0.5497077,
  0.5500037, 0.5503005, 0.5505979, 0.5508961, 0.5511999, 0.5515067, 0.5518149, 0.5521285, 0.5524459, 0.5527606, 0.5530767, 0.5533973, 0.5537209, 0.5540458, 0.5543711, 0.5547006, 0.5550332, 0.5553655, 0.5557021, 0.5560428,
  0.5563825, 0.5567272, 0.5570783, 0.5574313, 0.5577855, 0.5581408, 0.5584984, 0.558862, 0.5592301, 0.5595984, 0.5599681, 0.5603413, 0.5607177, 0.5610978, 0.5614823, 0.5618686, 0.5622579, 0.5626528, 0.5630517, 0.5634492,
  0.5638507, 0.5642599, 0.5646686, 0.5650796, 0.5654947, 0.5659142, 0.5663398, 0.5667692, 0.5672012, 0.5676378, 0.5680759, 0.5685167, 0.5689623, 0.5694101, 0.5698642, 0.5703221, 0.5707848, 0.5712521, 0.5717234, 0.5722005,
  0.5726816, 0.5731653, 0.5736523, 0.574144, 0.5746387, 0.5751385, 0.5756432, 0.576155, 0.5766714, 0.5771917, 0.5777192, 0.5782528, 0.578792, 0.5793384, 0.5798885, 0.5804407, 0.5809973, 0.5815596, 0.5821303, 0.5827099,
  0.5832958, 0.5838867, 0.584483, 0.585089, 0.585699, 0.5863112, 0.5869315, 0.5875583, 0.5881943, 0.5888352, 0.5894831, 0.5901427, 0.5908096, 0.5914807, 0.5921588, 0.592844, 0.5935381, 0.5942447, 0.5949571, 0.5956747,
  0.5964004, 0.5971388, 0.5978889, 0.5986463, 0.5994135, 0.6001914, 0.6009767, 0.6017719, 0.6025752, 0.6033866, 0.6042145, 0.6050507, 0.6058981, 0.6067617, 0.6076333, 0.6085131, 0.6094103, 0.610321, 0.6112424, 0.6121788,
  0.6131286, 0.6140946, 0.6150733, 0.6160645, 0.6170748, 0.618103, 0.6191439, 0.6201995, 0.6212746, 0.6223671, 0.6234825, 0.6246177, 0.6257715, 0.6269466, 0.6281416, 0.629364, 0.6306055, 0.6318621, 0.6331443, 0.6344519,
  0.6357835, 0.6371424, 0.6385294, 0.6399474, 0.6413939, 0.6428714, 0.6443793, 0.6459168, 0.6474937, 0.6491122, 0.6507658, 0.6524613, 0.6541951, 0.6559698, 0.6577994, 0.6596742, 0.6615943, 0.6635629, 0.6655833, 0.6676655,
  0.6698069, 0.6720175, 0.6743031, 0.676658, 0.679095, 0.6816108, 0.6842097, 0.6869088, 0.6897115, 0.6926211, 0.6956437, 0.6987869, 0.7020639, 0.7054881, 0.7090829, 0.7128501, 0.7167924, 0.7209545, 0.7253613, 0.7300176,
  0.7349495, 0.7378242, 0.737106, 0.7363889, 0.7356715, 0.7349546, 0.7342404, 0.733525, 0.7328098, 0.7320967, 0.7313842, 0.7306709, 0.7299576, 0.7292471, 0.7285383, 0.7278294, 0.7271212, 0.7264152, 0.7257108, 0.725008,
  0.7243064, 0.7236054, 0.7229066, 0.7248928, 0.7302105, 0.7355986, 0.7410725, 0.7456231, 0.7491655, 0.7527621, 0.7564233, 0.7587261, 0.7605307, 0.7623645, 0.7640161, 0.7641419, 0.7642802, 0.764431, 0.763568, 0.7621648,
  0.760762, 0.7591222, 0.7568917, 0.7546541, 0.7524019, 0.7490533, 0.7456876, 0.7423044, 0.7382382, 0.7339948, 0.7297378, 0.7250945, 0.7202464, 0.7153761, 0.7103242, 0.7051435, 0.6999506, };

  float Cd = Interpolation::Linear(machValues, cdValues, 435, mach, false);

  return Cd;
}

float get_thrust(float time) {
  // Range of validity
  if (time < 0) return 0;
  if (time > 4505) return 0;
  
  double timeValues[138] = { 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 170.0, 180.0, 190.0, 200.0, 210.0, 220.0, 230.0, 240.0, 250.0,
  260.0, 270.0, 280.0, 290.0, 300.0, 310.0, 320.0, 330.0, 340.0, 350.0, 360.0, 370.0, 380.0, 390.0, 400.0, 410.0, 420.0, 430.0, 440.0, 450.0, 460.0, 470.0, 480.0, 490.0, 500.0, 510.0, 520.0,
  530.0, 540.0, 550.0, 560.0, 570.0, 580.0, 595.0, 617.5, 651.25, 701.25, 751.25, 801.25, 851.25, 901.25, 951.25, 1001.25, 1051.25, 1101.25, 1151.25, 1201.25, 1251.25, 1301.25, 1351.25, 1401.25,
  1451.25, 1501.25, 1551.25, 1601.25, 1651.25, 1701.25, 1751.25, 1801.25, 1851.25, 1901.25, 1951.25, 2001.2500000000002, 2051.25, 2101.25, 2151.25, 2201.25, 2251.25, 2301.25, 2351.25, 2401.25,
  2451.25, 2501.25, 2551.25, 2601.25, 2651.25, 2701.25, 2751.25, 2801.25, 2851.25, 2901.25, 2951.25, 3001.25, 3051.25, 3101.25, 3151.25, 3201.25, 3251.25, 3301.25, 3351.25, 3401.25, 3451.25,
  3501.25, 3551.25, 3601.25, 3651.25, 3701.25, 3751.25, 3801.25, 3851.25, 3901.25, 3951.25, 4001.2499999999995, 4051.2499999999995, 4101.25, 4151.25, 4201.25, 4251.25, 4301.25, 4351.25, 4401.25,
  4451.25, 4500.0, };
  
  double thrustValues[138] = { 339.8160167, 1019.42805, 1699.0400833, 2042.0487833, 2048.45415, 2054.8595167, 2061.2492, 2067.6232, 2073.9972, 2080.3547333, 2086.6958, 2093.0368667, 2099.3607333,
  2105.6674, 2111.9740667, 2118.2627833, 2124.53355, 2130.8043167, 2137.0563833, 2143.28975, 2149.5231167, 2155.737, 2161.9314, 2168.1258, 2174.2999833, 2180.45395, 2186.6079167, 2192.7409333,
  2198.853, 2204.9650667, 2211.0554, 2217.124, 2223.1926, 2229.2387333, 2235.2624, 2241.2860667, 2247.2865167, 2253.26375, 2259.2409833, 2265.19425, 2271.12355, 2277.05285, 2282.95745, 2288.83735,
  2294.71725, 2300.5717, 2306.4007, 2312.2297, 2318.0325167, 2323.80915, 2329.5857833, 2335.3355, 2341.0583, 2346.7811, 2352.47625, 2358.14375, 2363.81125, 2369.4503667, 2376.4637833, 2386.9443227,
  2402.524232, 2425.4277436, 2452.3104615, 2478.6588333, 2504.4568386, 2529.6773209, 2554.3064204, 2578.3295037, 2601.7194624, 2624.464098, 2646.5501393, 2667.9508711, 2688.6559062, 2708.6534364,
  2727.9175613, 2746.4396303, 2764.2096134, 2781.20247, 2797.4116903, 2812.8289898, 2827.430463, 2841.2117614, 2854.1666208, 2866.2724081, 2877.5269184, 2887.925984, 2897.4486955, 2906.0950665,
  2913.8629294, 2920.7331752, 2926.708131, 2931.7879246, 2935.9551548, 2939.214659, 2941.5686274, 2943.0018719, 2943.5213758, 2943.1315693, 2941.8195173, 2939.5944389, 2936.4628529, 2932.4141909,
  2927.4597458, 2921.6083189, 2914.8515721, 2907.2029089, 2898.6732123, 2889.2564657, 2878.968063, 2867.8208057, 2855.8111141, 2633.0141, 1472.6843296, 1430.3205992, 1423.3682252, 1416.1251483,
  1408.5908993, 1400.7721073, 1392.6741645, 1384.2971722, 1375.6479247, 1366.7321927, 1357.178467, 831.0515378, 630.0208386, 626.834939, 623.5596734, 620.1964339, 616.7449985, 613.207164, 609.5844058,
  605.8765364, 602.0854696, 598.2127745, 594.2582801, 590.2239678, 586.1114558, 403.5401553, };

  float thrust = Interpolation::Linear(timeValues, thrustValues, 138, time, true);

  return thrust;
}

float get_mass(float time) { // [ms]
  // Range of polynomial validity
  if (time < 0.0) return 27.996384;
  if (time > 4550.0) return 23.101488;

  // These constants are obtained from ../Utilities/mass_extractor.py using the OpenRocket simulation
  double consts[] = {
    -3.5082664748532575e-50,
    1.4709841712847682e-45,
    -2.661943862688579e-41,
    2.775533101334194e-37,
    -1.86656961222092e-33,
    8.553688506193746e-30,
    -2.743882096268725e-26,
    6.220125122250671e-23,
    -9.923472327981488e-20,
    1.0949174756638967e-16,
    -8.078494282902627e-14,
    3.761798077286006e-11,
    -9.95716331520372e-09,
    1.0343767409692512e-06,
    -0.001028587721742733,
    28.013001760150956,
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

float radians_to_degrees(float radians) {
  return (radians / 3.1415926535897932384626433832795) * 180;
}


// If needed: utility functions for deployment ratio <--> servo degrees <--> c_d
