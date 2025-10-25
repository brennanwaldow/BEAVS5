#ifndef DELCARES_H
#define DELCARES_H

#include "misc.h"

void log(String message);

void write_telemetry_headers();
void write_telemetry();
void collect_telemetry();
void calculate_telemetry();

void command_deflection(float deflection);

void preflight_loop(int core);
void disarmed_loop(int core);
void ready_loop(int core);
void overshoot_loop(int core);
void coast_loop(int core);
void descend_loop(int core);
void flight_loop(int core);

void boot();
void preflight();
void arm();
void disarm();
void launch();
void coast();
void descend();

void tick_PID();

void get_trolled_idiot();

float get_mass(float time);
float gravity(float altitude);
float get_speed_of_sound(float altitude);
float get_air_density(float altitude);
float degrees_to_radians(float degrees);
float radians_to_degrees(float radians);
float velocity_lookup();
float get_Cd(float mach);
float get_thrust(float time);

float get_Fd_BEAVS(float velocity, float Cd_BEAVS, float A_BEAVS,
                   float air_density);

#endif
