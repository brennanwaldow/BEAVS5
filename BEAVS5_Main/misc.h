#ifndef MISC_H
#define MISC_H

#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdint.h>
#include <string>

#define LOW 0x0
#define HIGH 0x1

#define INPUT 0x0
#define OUTPUT 0x1

#define LED_BUILTIN 13

std::string to_precision(double x, unsigned char decimalPlaces);

// This sucks
// The constructors should really be explicit, but whatever
class String : public std::string {
public:
  String() : std::string() {}
  String(const std::string &str) : std::string(str) {}
  String(const char *str) : std::string(str) {}

  String(char c) : std::string(1, c) {}
  String(unsigned char x) : std::string(std::to_string(x)) {}
  String(int x) : std::string(std::to_string(x)) {}
  String(unsigned int x) : std::string(std::to_string(x)) {}
  String(long x) : std::string(std::to_string(x)) {}
  String(unsigned long x) : std::string(std::to_string(x)) {}
  String(float x, unsigned char decimalPlaces = 2)
      : std::string(to_precision(x, decimalPlaces)) {}
  String(double x, unsigned char decimalPlaces = 2)
      : std::string(to_precision(x, decimalPlaces)) {}

  String operator+(const char *rhs) const;
  String operator+(const String &rhs) const;
};

class HardwareSerial_s {
private:
  bool began = false;

public:
  HardwareSerial_s() {};

  bool begin(unsigned long baud);

  void print(const String &str);
  void print(float x, unsigned char decimalPlaces);
  void println(const String &str);

  std::stringstream data_s;
};

struct Pin_s {
  uint8_t mode;
  uint8_t value;
};

unsigned long millis();
unsigned long micros();
void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
void delay(unsigned long);
void delayMicroseconds(unsigned long);

using std::atan;
using std::cos;
using std::pow;
using std::round;
using std::sin;
using std::sqrt;

const int pin_count_s = 29;
extern unsigned long long micros0_s;
extern unsigned long long micros1_s;
extern Pin_s pins_s[pin_count_s];

extern int cpu_s;
extern void (*delay_callback_s)();

extern HardwareSerial_s Serial;

#endif
