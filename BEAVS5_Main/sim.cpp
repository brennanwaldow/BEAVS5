#include "sim.h"
#include <algorithm>

// TODO: Consider adding more callbacks. It would be good to figure out a way to
// have sensor values changed even without delay because running code takes time
// and this could be simulated with more callbacks than just during delay. Or
// maybe in between loops. However, I think running code will be insignificant
// compared to sensor update rate so it wouldn't matter. Also there would have
// to be a way to estimate time during callbacks. Some functions like
// performReading and maybe writing to the SD should have delay though.
// TODO: Add noise simulation especially since some of the sensor have different
// sampling rates.
// TODO: Instead of using _s maybe use namespaces

// There may be a better way to do this. This is highly sus as push_type is not
// assignable for a reason
// The functions return immediatly because they are run on init of the cpu0 and
// cpu1 so the first call is just to init

void Sim_s::run0(boost::coroutines2::coroutine<void>::push_type &yield) {
  yield0 = &yield;
  yield();

  board_s.setup();
  while (true) {
    board_s.loop();
  }
}

void Sim_s::run1(boost::coroutines2::coroutine<void>::push_type &yield) {
  yield1 = &yield;
  yield();

  board_s.setup1();
  while (true) {
    board_s.loop1();
  }
}

void Sim_s::yield() {
  if (cpu == 0) {
    (*yield0)();
  } else {
    (*yield1)();
  }
}

void Sim_s::step() {
  if (micros0 <= micros1) {
    cpu = 0;
    cpu0();
  } else {
    cpu = 1;
    cpu1();
  }

  micros_s = std::min(micros0, micros1);
}

void Sim_s::step_to(unsigned long long time) {
  while (time > micros_s) {
    step();
  }
}

void Sim_s::step_by(unsigned long long time) { step_to(micros_s + time); }

unsigned long Sim_s::millis() { return micros() / 1000; }

unsigned long Sim_s::micros() {
  if (cpu == 0) {
    return micros0;
  } else {
    return micros1;
  }
}

void Sim_s::pinMode(uint8_t pin, uint8_t mode) {
  assert(pin < pin_count_s);
  assert(mode == OUTPUT || mode == INPUT);

  pins_s[pin].mode = mode;
}

void Sim_s::digitalWrite(uint8_t pin, uint8_t value) {
  assert(pin < pin_count_s);
  assert(value == LOW || value == HIGH);
  assert(pins_s[pin].mode == OUTPUT);

  pins_s[pin].value = value;
}

int Sim_s::digitalRead(uint8_t pin) {
  assert(pin < pin_count_s);
  assert(pins_s[pin].mode == INPUT);

  return pins_s[pin].value;
}

void Sim_s::delay(unsigned long value) { delayMicroseconds(value * 1000); }

void Sim_s::delayMicroseconds(unsigned long value) {
  if (cpu == 0) {
    micros0 += value;
  } else {
    micros1 += value;
  }

  yield();
}
