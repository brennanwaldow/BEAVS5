#ifndef WIRE_H
#define WIRE_H

class TwoWire {
private:
  bool began = false;

public:
  TwoWire();
  void begin();
};

extern TwoWire Wire;

#endif
