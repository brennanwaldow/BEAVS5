#ifndef BNO_H
#define BNO_H

#include <stdint.h>

#include "wire.h"

typedef enum {
  SENSOR_TYPE_ORIENTATION = (3),
  SENSOR_TYPE_LINEAR_ACCELERATION = (10),
} sensors_type_t;

typedef struct {
  union {
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
} sensors_vec_t;

typedef struct {
  int32_t type;
  sensors_vec_t acceleration;
  sensors_vec_t orientation;
} sensors_event_t;

// TODO: Finish this to specs
class Adafruit_BNO055 {
private:
  bool began = false;
  bool extCrystal = false;

  TwoWire *wire;

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
  bool getEvent(sensors_event_t *event, adafruit_vector_type_t type);
};

extern sensors_vec_t acc_state_s;
extern sensors_vec_t gyro_state_s;

#endif
