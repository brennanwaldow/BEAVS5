#ifndef WIRE_H
#define WIRE_H

class TwoWire {
private:
  bool began = false;

public:
  TwoWire() {};
  void begin();

  bool began_s() const;
};

extern TwoWire Wire;

#endif
