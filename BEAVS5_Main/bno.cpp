#include "bno.h"

#include "wire.h"
#include <cassert>

sensors_vec_t acc_state_s = {};
sensors_vec_t gyro_state_s = {};

Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID, uint8_t address,
                                 TwoWire *theWire) {
  assert(!began);
  assert(sensorID == 55);
  assert(address == 0x28);
  assert(theWire == &Wire);

  wire = theWire;
}

bool Adafruit_BNO055::begin() {
  assert(wire->began_s());

  began = true;

  return true;
}

void Adafruit_BNO055::setExtCrystalUse(bool usextal) {
  assert(began);

  extCrystal = usextal;
}

bool Adafruit_BNO055::getEvent(sensors_event_t *event,
                               adafruit_vector_type_t type) {
  assert(began);
  assert(extCrystal);

  switch (type) {
  case VECTOR_LINEARACCEL:
    event->type = SENSOR_TYPE_LINEAR_ACCELERATION;
    event->acceleration = acc_state_s;
    break;
  case VECTOR_EULER:
    event->type = SENSOR_TYPE_ORIENTATION;
    event->orientation = gyro_state_s;
    break;
  }

  return true;
}
