#include "misc.h"

#include <cassert>
#include <iostream>
#include <ostream>

bool HardwareSerial_s::begin(unsigned long baud) {
  assert(!began);
  assert(baud == 115200);

  began = true;
  return true;
}

void HardwareSerial_s::print(const String &str) { std::cout << str; }

void HardwareSerial_s::println(const String &str) {
  std::cout << str << std::endl;
}

HardwareSerial_s Serial;
