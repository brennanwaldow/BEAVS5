#include "misc.h"

#include <cassert>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <ostream>

std::string to_precision(double x, unsigned char decimalPlaces) {
  std::stringstream stream;
  stream << std::fixed << std::setprecision(decimalPlaces) << x;
  return stream.str();
}

String String::operator+(const String &rhs) const {
  return String(static_cast<const std::string &>(*this) + rhs);
}

String String::operator+(const char *rhs) const {
  return String(static_cast<const std::string &>(*this) + rhs);
}

bool HardwareSerial_s::begin(unsigned long baud) {
  assert(!began);
  assert(baud == 115200);

  began = true;
  return true;
}

void HardwareSerial_s::print(const String &str) { data_s << str; }

void HardwareSerial_s::print(float x, unsigned char decimalPlaces) {
  data_s << String(x, decimalPlaces);
}

void HardwareSerial_s::println(const String &str) {
  // I think the \r\n is correct over endl, but IDK
  data_s << str << "\r\n";
}

HardwareSerial_s Serial;
