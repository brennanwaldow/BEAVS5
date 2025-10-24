#ifndef BNO_H
#define BNO_H

#include <stdint.h>

#include "wire.h"

typedef struct {
  union {
    float v[3];
    struct {
      float x;
      float y;
      float z;
    };
    struct {
      float roll;
      float pitch;
      float heading;
    };
  };
  int8_t status;
  uint8_t reserved[3];
} sensors_vec_t;

typedef struct {
  int32_t type;
  sensors_vec_t orientation;
  sensors_vec_t gyro;
} sensors_event_t;

class Adafruit_BNO055 {
public:
  typedef enum {
    BNO055_EULER_H_LSB_ADDR = 0X1A,
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
  } adafruit_bno055_reg_t;

  typedef enum {
    VECTOR_EULER = BNO055_EULER_H_LSB_ADDR,
    VECTOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
  } adafruit_vector_type_t;

  Adafruit_BNO055(int32_t sensorID, uint8_t address, TwoWire *theWire = &Wire);

  bool begin();
  void setExtCrystalUse(bool usextal);
  bool getEvent(sensors_event_t *, adafruit_vector_type_t);
};

#endif
