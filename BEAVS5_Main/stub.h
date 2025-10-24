#ifndef STUB_H
#define STUB_H

// TODO: change ifndef
#ifndef SIMULATION
#include "bmp.h"
#include "bno.h"
#include "sdfat.h"
#include "spi.h"
#include "wire.h"
#else
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#endif

#endif
