#ifndef SERIAL_H
#define SERIAL_H

#include <string>

typedef std::string String;

class HardwareSerial_s {
private:
  bool began = false;

public:
  HardwareSerial_s() {};

  bool begin(unsigned long baud);

  void print(const String &str);
  void println(const String &str);
};

extern HardwareSerial_s Serial;

#endif
