#include "bmp.h"

#include "wire.h"
#include <cassert>

bool Adafruit_BMP3XX::begin_I2C(uint8_t addr, TwoWire *theWire) {
  assert(!began);
  assert(addr == 0x77);
  assert(theWire == &Wire);
  assert(theWire->began_s());

  began = true;
  return true;
}

float Adafruit_BMP3XX::readTemperature() {
  assert(began);

  return 0.0F;
}

float Adafruit_BMP3XX::readPressure(void) {
  assert(began);

  return 0.0F;
}

float Adafruit_BMP3XX::readAltitude(float seaLevel) {
  assert(began);

  return 0.0F;
}

bool Adafruit_BMP3XX::setTemperatureOversampling(uint8_t os) {
  assert(began);

  return true;
}

bool Adafruit_BMP3XX::setPressureOversampling(uint8_t os) {
  assert(began);

  return true;
}

bool Adafruit_BMP3XX::setIIRFilterCoeff(uint8_t fs) {
  assert(began);

  return true;
}

bool Adafruit_BMP3XX::setOutputDataRate(uint8_t odr) {
  assert(began);

  return true;
}

bool Adafruit_BMP3XX::performReading() {
  assert(began);

  return true;
}
