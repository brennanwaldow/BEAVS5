#include "BEAVS5_Main.ino"

int main(int argc, char *argv[]) {
  setup();
  setup1();

  while (true) {
    loop();
    loop1();
  }

  return 0;
}
