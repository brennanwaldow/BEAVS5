/*
Hardware:
---
Computer: Raspberry Pi Pico
Altimeter: xxx
Accelerometer: xxx
*/

// -----   Libraries   -----



// -----   Global Variables   -----

// Initialization
float launch_altitude = 0; // [meters]
float launch_altimeter = 1013.25; // [HPa]
float target_apogee = feet_to_meters(10000); // [meters], AGL

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

}

// -----   Control Loop   -----
void loop() {
    if (flight_phase == 0) { preflight_loop(); }
    else if (flight_phase == 1) { ready_loop(); }
    else if (flight_phase == 2) { flight_loop(); }
    else if (flight_phase == 3) { coast_loop(); }
}


void preflight_loop() {

}

void ready_loop() {

}

void flight_loop() {

}

void coast_loop() {

}



// -----   Phase Changeovers   -----
void arm() {
    // SAFETY PIN REMOVED: Arm Beavs and initiate startup
}

void coast() {
    // ENGINE CUTOFF: Deploy BEAVS
}

void descend() {
    // APOGEE REACHED: BEAVS safing, PID shutdown
}



// -----   Functions   -----

// Tick PID controller
void PID() {

}

void command_deflection(float deflection) {  // [ratio], 0 to 1

}

// Velocity lookup table
void velocity_lookup() {

}



// Utility

float feet_to_meters(float length) {     // length [feet]
    return length / 3.281; // [meters]
}

float meters_to_feet(float length) {     // length [meters]
    return length * 3.281; // [feet]
}
