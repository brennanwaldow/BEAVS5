#include "wire.h"

#include <cassert>

void TwoWire::begin() {
  assert(!began);
  began = true;
}

bool TwoWire::began_s() const { return began; }

TwoWire Wire;
