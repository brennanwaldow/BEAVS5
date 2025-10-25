#include "BEAVS5_Main.ino"

// TODO: Consider adding more callbacks. It would be good to figure out a way to
// have sensor values changed even without delay because running code takes time
// and this could be simulated with more callbacks than just during delay.
// However, I think running code will be insignificant compared to sensor update
// rate so it wouldn't matter. Also there would have to be a way to estimate
// time during callbacks. Some functions like performReading and maybe writing
// to the SD should have delay though.
// TODO: Add noise simulation especially since some of the sensor have different
// sampling rates.
int main(int argc, char *argv[]) {
  setup();
  setup1();

  while (true) {
    loop();
    loop1();
  }

  return 0;
}
