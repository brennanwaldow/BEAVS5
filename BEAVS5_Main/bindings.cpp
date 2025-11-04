#include <cassert>
#include <pybind11/pybind11.h>

#include "bmp.h"
#include "bno.h"
#include "board.h"
#include "misc.h"
#include "pybind11/pytypes.h"
#include "sim.h"

PYBIND11_MODULE(beavs_sim, mod, pybind11::mod_gil_not_used()) {
  pybind11::class_<HardwareSerial_s>(mod, "Serial")
      .def("contents", [](const HardwareSerial_s &serial) {
        return pybind11::bytes(serial.data_s.str());
      });

  // C++ pointer to member syntax is lacking
  pybind11::class_<Adafruit_BNO055>(mod, "BNO")
      .def_property(
          "acc_x", [](Adafruit_BNO055 &bno) { return bno.acc_state_s.x; },
          [](Adafruit_BNO055 &bno, float acc_x) { bno.acc_state_s.x = acc_x; })
      .def_property(
          "acc_y", [](Adafruit_BNO055 &bno) { return bno.acc_state_s.y; },
          [](Adafruit_BNO055 &bno, float acc_y) { bno.acc_state_s.x = acc_y; })
      .def_property(
          "acc_z", [](Adafruit_BNO055 &bno) { return bno.acc_state_s.z; },
          [](Adafruit_BNO055 &bno, float acc_z) { bno.acc_state_s.z = acc_z; })
      .def_property(
          "heading",
          [](Adafruit_BNO055 &bno) { return bno.acc_state_s.heading; },
          [](Adafruit_BNO055 &bno, float heading) {
            bno.gyro_state_s.heading = heading;
          })
      .def_property(
          "pitch", [](Adafruit_BNO055 &bno) { return bno.acc_state_s.pitch; },
          [](Adafruit_BNO055 &bno, float pitch) {
            bno.gyro_state_s.pitch = pitch;
          })
      .def_property(
          "roll", [](Adafruit_BNO055 &bno) { return bno.acc_state_s.roll; },
          [](Adafruit_BNO055 &bno, float roll) {
            bno.gyro_state_s.roll = roll;
          });

  pybind11::class_<Adafruit_BMP3XX>(mod, "BMP")
      .def_readwrite("tempurature", &Adafruit_BMP3XX::tempurature_s)
      .def_readwrite("pressure", &Adafruit_BMP3XX::pressure_s)
      .def_readwrite("altitude", &Adafruit_BMP3XX::altitude_s);

  pybind11::class_<Board>(mod, "Board")
      .def_readonly("serial", &Board::Serial)
      .def_readonly("bno", &Board::bno)
      .def_readonly("bmp", &Board::bmp);

  pybind11::class_<Sim_s>(mod, "Sim")
      .def(pybind11::init<>())
      .def("step_to", &Sim_s::step_to)
      .def("step_by", &Sim_s::step_by)
      .def("step", &Sim_s::step)
      .def("set_pin",
           [](Sim_s &sim, uint8_t pin, bool value) {
             assert(pin < pin_count_s);
             assert(value == LOW || value == HIGH);
             assert(sim.pins_s[pin].mode == INPUT);

             sim.pins_s[pin].value = value;
           })
      .def_readonly("micros", &Sim_s::micros_s)
      .def_readonly("board", &Sim_s::board_s);
}
