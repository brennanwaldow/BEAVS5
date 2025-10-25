#include "BEAVS5_Main.h"
#include "misc.h"
#include <boost/coroutine2/all.hpp>
#include <boost/coroutine2/coroutine.hpp>
#include <boost/coroutine2/detail/pull_coroutine.hpp>
#include <cassert>
#include <csetjmp>
#include <cstdio>

// TODO: Consider adding more callbacks. It would be good to figure out a way to
// have sensor values changed even without delay because running code takes time
// and this could be simulated with more callbacks than just during delay. Or
// maybe in between loops. However, I think running code will be insignificant
// compared to sensor update rate so it wouldn't matter. Also there would have
// to be a way to estimate time during callbacks. Some functions like
// performReading and maybe writing to the SD should have delay though.
// TODO: Add noise simulation especially since some of the sensor have different
// sampling rates.

void (*callback)(unsigned long long time) = nullptr;

// There may be a better way to do this. This is highly sus as push_type is not
// assignable for a reason
// The functions return immediatly because they are run on init of the cpu0 and
// cpu1 so the first call is just to init
boost::coroutines2::coroutine<void>::push_type *yield0;
boost::coroutines2::coroutine<void>::push_type *yield1;

void run0(boost::coroutines2::coroutine<void>::push_type &yield) {
  yield0 = &yield;
  yield();

  setup();
  while (true) {
    loop();
  }
}

void run1(boost::coroutines2::coroutine<void>::push_type &yield) {
  yield1 = &yield;
  yield();

  setup1();
  while (true) {
    loop1();
  }
}

void ret() {
  if (cpu_s == 0) {
    (*yield0)();
  } else {
    (*yield1)();
  }
}

void run() {
  assert(callback != nullptr);

  delay_callback_s = ret;

  boost::coroutines2::coroutine<void>::pull_type cpu0(run0);
  boost::coroutines2::coroutine<void>::pull_type cpu1(run1);
  while (true) {
    if (micros0_s <= micros1_s) {
      callback(micros0_s);

      cpu_s = 0;
      cpu0();
    } else {
      callback(micros1_s);

      cpu_s = 1;
      cpu1();
    }
  }
}
