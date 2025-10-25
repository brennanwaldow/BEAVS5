#ifndef LOOP_H
#define LOOP_H

void step();

void step_to(unsigned long long time);

void step_by(unsigned long long time);

void yield();

extern unsigned long long micros_s;

#endif
