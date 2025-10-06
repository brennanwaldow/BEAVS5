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
enum { STOWED, ZEROING, MAX_BRAKING, DATA_COLLECTION, ACTIVE };
    // STOWED -- Flight computer runs, BEAVS blades remain flush with Outer Diameter
    // ZEROING -- For blade installation / integration, blade gear returns to zero position at Inner Diameter
    // ACTIVE -- PID loop controls blade deflection
enum { PIN_ARMING, TIMER_ARMING };
    // PIN_ARMING -- Remove Before Flight pins dictate arming cycle
    // TIMER_ARMING -- Timer automatically engages ARMED phase after boot, ONLY to be used for testing / simulation

// TODO: SET TO FIELD/ACTIVE/PIN_ARMING before COMPETITION FLIGHT; or FIELD/DATA_COLLECTION/PIN_ARMING for subscale drag validation flight
int BEAVS_mode = SIM;
int BEAVS_control = STOWED;
int BEAVS_arming = TIMER_ARMING;

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
int servo_mosfet_pin = 27; // GPIO 27 / Physical Pin 32

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

// long launch_clock = 0;
long last_reset = 0;

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

// DATA_COLLECTION mode
long datacoll_timer = 0;                // [ms]
float datacoll_time_interval = 2;       // [s]
float datacoll_extension = 12.5;        // [%]

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

  pinMode(servo_mosfet_pin, OUTPUT);

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
    if (BEAVS_arming == TIMER_ARMING) {
      if (millis() > 3000) arm();
    } else {
      // TODO: Add conditions for awaiting pin interrupt here {
        // arm();
      // }
    }

    collect_telemetry();
    calculate_telemetry();
    // TODO: disable telemetry write on ground for final flight
    write_telemetry();
  } else if (core == 2) {

  }
}

void ready_loop(int core) {
  if (core == 1) {
    // TODO: Conditions for awaiting interrupt for reinsertion of Remove Before Flight Pin
      // disarm();
    // }

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
    } else if (BEAVS_control == DATA_COLLECTION) {
      if (millis() - datacoll_timer > (datacoll_time_interval * 1000)) {
        datacoll_timer = millis();
        u = u + (datacoll_extension / 100);
        command_deflection(u);
      }
    }

    if (height > target_apogee) overshoot();

    // TODO the altitude reading is TOO NOISY for this cutoff: will have to add additional conditions
    if (max_height > height && (millis() - max_height_clock) > 500) {
      descend();
    }
  } else if (core == 2) {
    if (BEAVS_control == ACTIVE) {

    }
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

      // Unpower servo to aid in de-integration
      digitalWrite(servo_mosfet_pin, LOW);
    }
  } else if (core == 2) {

  }
}



// -----   Phase Changeovers   -----
void arm() {
  // SAFETY PIN REMOVED: Arm BEAVS monitoring and initiate startup
  flight_phase = ARMED;

  log("Safety pin removed. BEAVS arming.");
  
  // Power servo MOSFET
  digitalWrite(servo_mosfet_pin, HIGH);

  if (BEAVS_control == STOWED) {
    log("BEAVS control: STOWED.");
    command_deflection(0);
  } else if (BEAVS_mode == FIELD) {
    if (BEAVS_control == DATA_COLLECTION) {
      log("BEAVS control mode: DATA COLLECTION.");
      command_deflection(1);
      delay(2000);
      command_deflection(0.5);
      delay(1000);
      command_deflection(0);
      delay(3000);
      command_deflection(1);
      delay(2000);
      command_deflection(0);
    } else if (BEAVS_control == ACTIVE) {
      log("BEAVS control mode: ACTIVE.");
      command_deflection(1);
      delay(2000);
      command_deflection(0.5);
      delay(1000);
      command_deflection(0);
    }
  } else if (BEAVS_mode == SIM) {
    log("BEAVS running in SIM mode.");
    if (BEAVS_control == ZEROING) {
      command_deflection(-1);
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
  log("Altitude achieved: " + String(max_height, 2) + " m AGL   //   " + String(meters_to_feet(max_height), 2) + " ft AGL");

  apogee_timestamp = millis();

  // Funny moment for expo display

  // last_reset = millis();
  // arm();
  // velocity = 0;
  // altitude = launch_altitude;
  // max_height = 0;
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

    // TODO: look more at this
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
  float A_ref = 0.009;
  float virtual_deflection = max((virtual_angle - 4.322) / (120 - 4.322), 0);
  float A_beavs = ((feet_to_meters(1 / 12) * feet_to_meters(1.825 / 12)) * 2) * virtual_deflection;
  
  float speed_of_sound = (-0.003938991248485773 * altitude) + 345.82471162249857;
  float Mach = abs(velocity) / speed_of_sound;
  float Cd_rocket = get_Cd(Mach);
  float air_density = (-4.224962710944224e-14 * pow(altitude, 3) + 4.1628470681419666e-09 * pow(altitude, 2) + (-0.00011736669958683132 * altitude) + 1.2251486249604124);

  float Cd_beavs = 4.8 * (sqrt(A_beavs / A_ref)) * blade_modulation;
  float Cd = Cd_rocket + (Cd_beavs * (A_beavs / A_ref));

  drag_force_expected = abs(0.5 * air_density * (velocity * velocity) * Cd * A_ref);

  // Convert two-axis pitch rotation into single-axis pitch, degrees from vertical
  float a1 = degrees_to_radians(pitch_angle_x);
  float a2 = degrees_to_radians(pitch_angle_y);

  float h1 = cos(a1);
  float l1 = sin(a1);

  float l2 = h1 * sin(a2);
  float h = h1 * cos(a2);

  float l = sqrt((l1 * l1) + (l2 * l2));

  // funny thing to make sure i dont divide by zero during flight lmfao
  if (h = 0) {
    // if h = 0, angle vector is perfectly aligned with x/y plane, meaning it is 90 degrees rotated from z-axis vertical
    pitch_angle = 90;
  } else {
    pitch_angle = abs(radians_to_degrees(atan(l / h)));
  }

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
  float min_def = 4.322;
  float max_def = 120.0;
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
  }
}

// Velocity lookup table
// ONLY VALID for launch site: BROTHERS
float velocity_lookup() {
  // Range of polynomial validity: Burnout to apogee
  if (height < 776.2875822000002) return 283.67155136563815;
  if (height > target_apogee) return 0;

  // These constants are obtained from ../Utilities/OpenRocket Extractors/lookup_table.py using the OpenRocket simulation
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


//    ---   ON-BOARD SIMULATION   ---

// Spoof flight computer data with real-time simulated flight
void get_trolled_idiot() {
  // Offset sim clock by 15s to allow for startup
  long launch_clock = millis() - 15000 - last_reset;

  // Calculate atmospherics
  float speed_of_sound = (-0.003938999995558203 * altitude) + 345.82471162249857;      // Obtain constants from Utilities/OpenRocket Extractors/speed_of_sound_extractor.py
  float Mach = abs(velocity) / speed_of_sound;
  float Cd_rocket = get_Cd(Mach);
  float air_density = (-4.224962710944224e-14 * pow(altitude, 3) + 4.1628470681419666e-09 * pow(altitude, 2) + (-0.00011736669958683132 * altitude) + 1.2251486249604124);   // Obtain constants from Utilities/OpenRocket Extractors/air_density_extractor.py

  // Mass and thrust as a function of time - changing with motor burn
  float mass = get_mass(launch_clock);
  float thrust = get_thrust(launch_clock);

  // Modulate thrust to simulate performance deviation in reality
  thrust = thrust * thrust_modulation;

  // Change in time [seconds]
  double dt = (curr_time - clock_time) / (double) 1000000;

  // After T-0, calculate acceleration from applied forces
  if (launch_clock > 0) {
    acceleration = -(gravity(altitude) * cos(degrees_to_radians(launch_angle))) + (thrust / mass);
    perpendicular_acceleration = (gravity(altitude) * sin(degrees_to_radians(launch_angle)));
  }

      // --- Drag ---
  
  // Flip drag vector if velocity is negative so that it's always a dissipative force
  int dir = 1;
  if (velocity < 0) dir = -1;

  // TODO: Update area for new Outer Diameter when openrocket finalized
  float A_ref = 0.009;                                                                                // Cross-section area of fuselage corresponding to OpenRocket reference area
  float virtual_deflection = max((virtual_angle - 4.322) / (120 - 4.322), 0);                                 // Map blade deflection angle to a [0, 1] ratio for math
  float A_beavs = ((feet_to_meters(1 / 12) * feet_to_meters(1.825 / 12)) * 2) * virtual_deflection;     // Cross-section area of exposed BEAVS blades
  // TODO: Adjust rudimentary drag equation with correction factors from Ansys Fluent comparison - using Utilities/drag_mapper.py
  float Cd_beavs = 4.8 * (sqrt(A_beavs / A_ref)) * blade_modulation;
  float Cd = Cd_rocket + (Cd_beavs * (A_beavs / A_ref));                        // Total combined rocket drag coefficient

  // Calculate drag force and reapply to total instantaneous acceleration
  float Fd = (0.5 * air_density * (velocity * velocity) * Cd * A_ref) * dir;
  acceleration = acceleration - (Fd / mass);

  // Calculate change in velocity from instantaneous acceleration
  double dv = acceleration * dt;
  double dv_perpendicular = perpendicular_acceleration * dt;

  if (launch_clock > 0) {
    // Adjust previous frame's instanteous velocity by the change in velocity
    velocity = velocity + dv;
    perpendicular_velocity = perpendicular_velocity + dv_perpendicular;
  }

  // Calculate change in altitude from instantaneous velocity
  double dh = ((velocity * dt) * cos(degrees_to_radians(launch_angle)));  // From rotated coordinate system to vertical coordinate system
  // Adjust previous frame's instantaneous altitude by change in altitude
  altitude = altitude + dh;
  if (altitude < launch_altitude) altitude = launch_altitude;     // oops don't fall through the floor !


  // Uncomment this block to fix flight conditions to post-launch without simulating burn:
      // Hardcoded data from OpenRocket export at time of burnout
      // Sim will hold until burnout, then release back to live sim as if engine burn is already complete

  // if (launch_clock < (4.505) * 1000) {       // t_burnout = 4.505s (4505 ms)
  //   altitude = 796.68;
  //   acceleration = 0;
  //   velocity = 285.398;
  // } else {
  //   flight_phase = COAST;
  // }


  // Smoothly adjust physical real-world blade angle towards the commanded angle based on measured servo response speed (150deg/1.41s)
  if (commanded_angle > virtual_angle) virtual_angle = virtual_angle + min(commanded_angle - virtual_angle, (150.0 / 1.41) * dt);
  else if (commanded_angle < virtual_angle) virtual_angle = virtual_angle + max(commanded_angle - virtual_angle, -(150.0 / 1.41) * dt);

  // the big data dump
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
  Serial.print(virtual_angle);
  Serial.print(" ");
  Serial.println(flight_phase);


          // Printing for csv output: not really needed anymore now that logging is available
  // Serial.print((float) launch_clock / 1000.0);
  // Serial.print(",");
  // Serial.print(acceleration);
  // Serial.print(",");
  // Serial.print(velocity);
  // Serial.print(",");
  // Serial.println(height);
}


// Data interpolation functions

float gravity(float altitude) {          // altitude [meters]
  // Range of polynomial validity
  if (altitude < 1380.00067880291) return 9.800601153885829;
  if (altitude > 6575.6628767) return 9.784637022898936;

  // These constants are obtained from ../Utilities/OpenRocket Extractors/gravity_extractor.py using the OpenRocket simulation
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
  if (mach > 1.117) return 1.073;

  if (mach < 0.101) {
    // Plateau drag curve at low speed on launch
    if (height < 150) return 0.601;

    if (mach < 0.012) return 0.725;
  }

  // These interpolation values are obtained from Utilities/OpenRocket Extractors/drag_curve.py
  double machValues[302] = { 0.101, 0.102, 0.104, 0.105, 0.107, 0.108, 0.11, 0.111, 0.113, 0.114, 0.116, 0.117, 0.119, 0.12, 0.122, 0.124, 0.125, 0.127, 0.128, 0.13, 0.131, 0.133, 0.134, 0.136, 0.137, 0.139, 0.141, 0.142, 
  0.144, 0.145, 0.147, 0.149, 0.15, 0.152, 0.153, 0.155, 0.157, 0.158, 0.16, 0.161, 0.163, 0.165, 0.166, 0.168, 0.169, 0.171, 0.173, 0.174, 0.176, 0.178, 0.179, 0.181, 0.183, 0.184, 0.186, 0.187, 0.189, 0.191, 0.192, 0.194, 
  0.196, 0.197, 0.199, 0.201, 0.203, 0.204, 0.206, 0.208, 0.209, 0.211, 0.213, 0.214, 0.216, 0.218, 0.22, 0.221, 0.223, 0.225, 0.227, 0.228, 0.23, 0.232, 0.234, 0.235, 0.237, 0.239, 0.241, 0.242, 0.244, 0.246, 0.248, 0.25, 
  0.251, 0.253, 0.255, 0.257, 0.259, 0.261, 0.262, 0.264, 0.266, 0.268, 0.27, 0.272, 0.274, 0.275, 0.277, 0.279, 0.281, 0.283, 0.285, 0.287, 0.289, 0.291, 0.293, 0.295, 0.297, 0.299, 0.301, 0.303, 0.305, 0.306, 0.308, 0.31, 
  0.313, 0.315, 0.317, 0.319, 0.321, 0.323, 0.325, 0.327, 0.329, 0.331, 0.333, 0.335, 0.337, 0.339, 0.341, 0.344, 0.346, 0.348, 0.35, 0.352, 0.354, 0.357, 0.359, 0.361, 0.363, 0.366, 0.368, 0.37, 0.372, 0.375, 0.377, 
  0.379, 0.381, 0.384, 0.386, 0.389, 0.391, 0.393, 0.396, 0.398, 0.401, 0.403, 0.405, 0.408, 0.41, 0.413, 0.415, 0.418, 0.42, 0.423, 0.425, 0.428, 0.431, 0.433, 0.436, 0.439, 0.441, 0.444, 0.447, 0.449, 0.452, 0.455, 0.458,
  0.46, 0.463, 0.466, 0.469, 0.472, 0.475, 0.478, 0.48, 0.483, 0.486, 0.489, 0.492, 0.496, 0.499, 0.502, 0.505, 0.508, 0.511, 0.514, 0.518, 0.521, 0.524, 0.527, 0.531, 0.534, 0.538, 0.541, 0.544, 0.548, 0.551, 0.555, 0.559,
  0.562, 0.566, 0.57, 0.573, 0.577, 0.581, 0.585, 0.589, 0.593, 0.597, 0.601, 0.605, 0.609, 0.613, 0.617, 0.622, 0.626, 0.63, 0.635, 0.639, 0.644, 0.648, 0.653, 0.658, 0.663, 0.668, 0.672, 0.677, 0.682, 0.688, 0.693, 0.698, 
  0.703, 0.709, 0.714, 0.72, 0.726, 0.732, 0.737, 0.743, 0.749, 0.756, 0.762, 0.768, 0.775, 0.782, 0.788, 0.795, 0.802, 0.81, 0.817, 0.825, 0.832, 0.84, 0.848, 0.856, 0.865, 0.873, 0.882, 0.891, 0.901, 0.91, 0.92, 0.93, 0.941, 
  0.951, 0.962, 0.974, 0.986, 0.998, 1.01, 1.023, 1.037, 1.051, 1.065, 1.08, 1.093, 1.101, 1.107, 1.111, 1.114, 1.116, 1.117, };

  double cdValues[302] = { 0.601, 0.601, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.599, 0.599, 0.599, 0.599, 0.599, 0.599, 0.599, 0.599, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598,
  0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.598, 0.599, 0.599, 0.599, 0.599, 0.599, 0.599, 0.599, 0.599, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.601, 0.601, 0.601, 0.601, 0.601, 0.601, 0.602, 
  0.602, 0.602, 0.602, 0.603, 0.603, 0.603, 0.603, 0.604, 0.604, 0.604, 0.604, 0.605, 0.605, 0.605, 0.605, 0.606, 0.606, 0.606, 0.607, 0.607, 0.607, 0.607, 0.608, 0.608, 0.608, 0.609, 0.609, 0.609, 0.61, 0.61, 0.61, 0.611,
  0.611, 0.612, 0.612, 0.612, 0.613, 0.613, 0.614, 0.614, 0.614, 0.615, 0.615, 0.616, 0.616, 0.617, 0.617, 0.618, 0.618, 0.618, 0.619, 0.619, 0.62, 0.62, 0.621, 0.621, 0.622, 0.622, 0.623, 0.623, 0.624, 0.624, 0.625, 0.626,
  0.626, 0.627, 0.627, 0.628, 0.628, 0.629, 0.63, 0.63, 0.631, 0.631, 0.632, 0.633, 0.633, 0.634, 0.635, 0.635, 0.636, 0.637, 0.637, 0.638, 0.639, 0.639, 0.64, 0.641, 0.642, 0.642, 0.643, 0.644, 0.645, 0.645, 0.646, 0.647,
  0.648, 0.648, 0.649, 0.65, 0.651, 0.652, 0.653, 0.653, 0.654, 0.655, 0.656, 0.657, 0.658, 0.659, 0.66, 0.661, 0.662, 0.663, 0.664, 0.665, 0.666, 0.667, 0.668, 0.669, 0.67, 0.671, 0.672, 0.673, 0.674, 0.675, 0.676, 0.677,
  0.679, 0.68, 0.681, 0.682, 0.683, 0.685, 0.686, 0.687, 0.688, 0.69, 0.691, 0.692, 0.694, 0.695, 0.697, 0.698, 0.699, 0.701, 0.702, 0.704, 0.705, 0.707, 0.708, 0.71, 0.712, 0.713, 0.715, 0.717, 0.718, 0.72, 0.722, 0.724,
  0.726, 0.727, 0.729, 0.731, 0.733, 0.735, 0.737, 0.739, 0.741, 0.743, 0.746, 0.748, 0.75, 0.752, 0.755, 0.757, 0.759, 0.762, 0.764, 0.767, 0.77, 0.772, 0.775, 0.778, 0.781, 0.784, 0.787, 0.79, 0.793, 0.796, 0.799, 0.803,
  0.806, 0.809, 0.813, 0.817, 0.821, 0.825, 0.829, 0.834, 0.838, 0.843, 0.847, 0.852, 0.857, 0.863, 0.868, 0.874, 0.879, 0.885, 0.891, 0.898, 0.905, 0.912, 0.92, 0.927, 0.935, 0.944, 0.952, 0.961, 0.97, 0.98, 0.992, 
  1.004, 1.017, 1.03, 1.039, 1.046, 1.052, 1.058, 1.063, 1.067, 1.069, 1.071, 1.072, 1.072, 1.073, 1.073, 1.073, };

  float Cd = Interpolation::Linear(machValues, cdValues, 435, mach, false);

  return Cd;
}

float get_thrust(float time) {
  // Range of validity
  if (time < 0) return 0;
  if (time > 3000) return 0;
  
  // These interpolation values are obtained from Utilities/OpenRocket Extractors/thrust_extractor.py
  double timeValues[103] = { 0.0, 4.0, 11.0, 21.0, 31.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 170.0, 180.0, 190.0, 195.0, 202.0, 212.0, 227.0, 249.0, 281.0,
  296.0, 318.0, 351.0, 372.0, 403.0, 416.0, 435.0, 465.0, 508.0, 558.0, 600.0, 650.0, 700.0, 750.0, 788.0, 838.0, 888.0, 938.0, 988.0, 1038.0, 1066.0, 1108.0, 1158.0, 1208.0, 1258.0, 1261.0, 1266.0, 1272.0,
  1282.0, 1298.0, 1320.0, 1355.0, 1405.0, 1455.0, 1507.0, 1557.0, 1607.0, 1657.0, 1707.0, 1746.0, 1796.0, 1846.0, 1865.0, 1894.0, 1936.0, 1986.0, 1995.0, 2008.0, 2028.0, 2057.0, 2093.0, 2143.0, 2158.0, 2181.0,
  2214.0, 2263.0, 2313.0, 2363.0, 2389.0, 2428.0, 2478.0, 2528.0, 2577.0, 2627.0, 2677.0, 2693.0, 2717.0, 2753.0, 2761.0, 2773.0, 2791.0, 2818.0, 2858.0, 2908.0, 2958.0, 3000.0, };
  
  double thrustValues[103] = { 0.0, 10.664, 1268.961, 1287.347, 1305.732, 1322.279, 1316.087, 1309.896, 1303.704, 1297.512, 1291.32, 1285.129, 1278.937, 1272.745, 1266.553, 1260.362, 1254.17, 1247.978, 1241.786,
  1235.595, 1229.403, 1226.307, 1232.231, 1240.13, 1251.978, 1268.961, 1250.669, 1242.303, 1245.407, 1250.063, 1252.966, 1226.644, 1215.644, 1216.771, 1218.462, 1220.999, 1223.896, 1226.307, 1223.471, 1220.635, 1217.799,
  1215.644, 1209.89, 1204.136, 1198.383, 1192.629, 1186.875, 1183.653, 1180.208, 1176.107, 1172.005, 1167.904, 1167.658, 1166.878, 1165.707, 1163.952, 1161.318, 1157.368, 1151.443, 1142.774, 1134.104, 1125.004, 1117.196,
  1109.388, 1101.58, 1093.771, 1087.681, 1071.999, 1056.318, 1050.359, 1049.19, 1047.437, 1045.386, 1045.027, 1027.175, 1000.397, 960.231, 911.733, 784.591, 746.448, 705.317, 643.621, 554.504, 495.262, 436.021, 405.215,
  360.973, 304.252, 247.531, 191.944, 145.98, 100.016, 85.308, 70.254, 47.672, 42.654, 40.512, 37.3, 32.481, 25.253, 16.33, 7.406, 0.0, };

  float thrust = Interpolation::Linear(timeValues, thrustValues, 138, time, true);

  return thrust;
}

float get_mass(float time) { // [ms]
  // Range of polynomial validity
  if (time < 0.0) return 7.976;
  if (time > 3000.0) return 6.576;

  // These constants are obtained from ../Utilities/OpenRocket Extractors/mass_extractor.py using the OpenRocket simulation
  double consts[] = {
    1.9982731609414278e-47,
    -4.337053027546156e-43,
    4.22812688873407e-39,
    -2.4468506970588034e-35,
    9.360444333087315e-32,
    -2.4950590471432615e-28,
    4.759719269345194e-25,
    -6.571765614846873e-22,
    6.566409907308671e-19,
    -4.699987435987095e-16,
    2.360378981188635e-13,
    -8.039864996231402e-11,
    1.7585314845980733e-08,
    -2.2171119079858655e-06,
    -0.0005257900955843976,
    7.977756150682098,
  };

  int poly_order = 15;
  double result = 0;

  // pain
  for (int i = 0; i < poly_order + 1; i++) {
    result = result + ((double) pow(time, poly_order - i) * consts[i]);
  }

  return (float) result;
}


// Utility functions

float get_Fd_BEAVS(float velocity, float Cd_BEAVS, float A_BEAVS, float air_density) {
  return 0.5 * air_density * (velocity * velocity) * Cd_BEAVS * A_BEAVS;
}

float degrees_to_radians(float degrees) {
  return (degrees * 3.1415926535897932384626433832795) / 180;
}

float radians_to_degrees(float radians) {
  return (radians / 3.1415926535897932384626433832795) * 180;
}


// If needed: utility functions for deployment ratio <--> servo degrees <--> c_d?
