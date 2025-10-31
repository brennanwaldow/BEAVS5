#ifndef BMP_H
#define BMP_H

#include <stdint.h>

#include "wire.h"

#define BMP3_OVERSAMPLING_8X UINT8_C(0x03)
#define BMP3_OVERSAMPLING_16X UINT8_C(0x04)
#define BMP3_IIR_FILTER_COEFF_3 UINT8_C(0x02)
#define BMP3_ODR_50_HZ UINT8_C(0x02)

// TODO: Finish this to specs. Currently all the non-read functions do nothing
// and the performReading should call block based on the settings input.
class Adafruit_BMP3XX {
  bool began = false;

public:
  Adafruit_BMP3XX() {}

  float tempurature_s = 0.0F;
  float pressure_s = 0.0F;
  float altitude_s = 0.0F;

  bool begin_I2C(uint8_t addr, TwoWire *theWire = &Wire);
  float readTemperature();
  float readPressure();
  float readAltitude(float seaLevel);

  bool setTemperatureOversampling(uint8_t os);
  bool setPressureOversampling(uint8_t os);
  bool setIIRFilterCoeff(uint8_t fs);
  bool setOutputDataRate(uint8_t odr);

  bool performReading();
};

extern float tempurature_s;
extern float pressure_s;
extern float altitude_s;

#endif
