#include "misc.h"

#include <cassert>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <ostream>

unsigned long long micros0_s = 0;
unsigned long long micros1_s = 0;
Pin_s pins_s[pin_count_s] = {};

int cpu_s = 0;
void (*delay_callback_s)() = nullptr;

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

unsigned long millis() { return micros() / 1000; }

unsigned long micros() {
  if (cpu_s == 0) {
    return micros0_s;
  } else {
    return micros1_s;
  }
}

void pinMode(uint8_t pin, uint8_t mode) {
  assert(pin < pin_count_s);
  assert(mode == OUTPUT || mode == INPUT);

  pins_s[pin].mode = mode;
}

void digitalWrite(uint8_t pin, uint8_t value) {
  assert(pin < pin_count_s);
  assert(value == LOW || value == HIGH);
  assert(pins_s[pin].mode == OUTPUT);

  pins_s[pin].value = value;
}

int digitalRead(uint8_t pin) {
  assert(pin < pin_count_s);
  assert(pins_s[pin].mode == INPUT);

  return pins_s[pin].value;
}

void delay(unsigned long value) { delayMicroseconds(value * 1000); }

void delayMicroseconds(unsigned long value) {
  if (cpu_s == 0) {
    micros0_s += value;
  } else {
    micros1_s += value;
  }

  if (delay_callback_s != nullptr) {
    delay_callback_s();
  }
}

HardwareSerial_s Serial;
