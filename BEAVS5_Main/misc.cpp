#include "misc.h"

#include <cassert>
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

void HardwareSerial_s::print(const String &str) { std::cout << str; }

void HardwareSerial_s::print(float x, unsigned char decimalPlaces) {
  std::cout << String(x, decimalPlaces);
}

void HardwareSerial_s::println(const String &str) {
  std::cout << str << std::endl;
}

unsigned long millis() { return 0; }

unsigned long micros() { return 0; }

void pinMode(uint8_t, uint8_t) {}

void digitalWrite(uint8_t, uint8_t) {}

int digitalRead(uint8_t) { return 0; }

void delay(unsigned long) {}

void delayMicroseconds(unsigned long) {}

HardwareSerial_s Serial;
