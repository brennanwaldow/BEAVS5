#ifndef STUB_H
#define STUB_H

// TODO: change ifndef
#ifndef SIMULATION
#include "InterpolationLib.h"
#include "bmp.h"
#include "bno.h"
#include "declares.h"
#include "misc.h"
#include "sdfat.h"
#include "wire.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#else
#include "InterpolationLib.h"
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <utility/imumaths.h>
#endif

#endif
