#include "loop.h"
#include "misc.h"
#include <cstdio>
#include <iostream>
#include <ostream>

int main(int argc, char *argv[]) {
  callback = [](unsigned long long time) {
    std::cout << time << " " << micros0_s << " " << micros1_s << std::endl;
  };

  run();

  return 0;
}
