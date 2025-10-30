#include <pybind11/pybind11.h>

#include "board.h"
#include "misc.h"
#include "pybind11/pytypes.h"
#include "sim.h"

PYBIND11_MODULE(beavs_sim, mod, pybind11::mod_gil_not_used()) {
  pybind11::class_<HardwareSerial_s>(mod, "Serial")
      .def("contents", [](const HardwareSerial_s &serial) {
        return pybind11::bytes(serial.data_s.str());
      });

  pybind11::class_<Board>(mod, "Board").def_readonly("serial", &Board::Serial);

  pybind11::class_<Sim_s>(mod, "Sim")
      .def(pybind11::init<>())
      .def("step_to", &Sim_s::step_to)
      .def("step_by", &Sim_s::step_by)
      .def("step", &Sim_s::step)
      .def_readonly("micros", &Sim_s::micros_s)
      .def_readonly("board", &Sim_s::board_s);
}
