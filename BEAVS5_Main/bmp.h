#ifndef BMP_H
#define BMP_H

#include <stdint.h>

#include "wire.h"

class Adafruit_BMP3XX {
public:
  Adafruit_BMP3XX();

  bool begin_I2C(uint8_t addr, TwoWire *theWire = &Wire);
  float readTemperature(void);
  float readPressure(void);
  float readAltitude(float seaLevel);

  bool setTemperatureOversampling(uint8_t os);
  bool setPressureOversampling(uint8_t os);
  bool setIIRFilterCoeff(uint8_t fs);
  bool setOutputDataRate(uint8_t odr);
};

#endif
