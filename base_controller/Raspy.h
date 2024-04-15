#ifndef RASPY_h
#define RASPY_h

#include "Movement.h"

class Raspy{
  public: 
  Raspy(BNO *bno, LineSensor *line, ColorSensor *color, Movement *robot);
  executeCommand(uint8_t packet_size, uint8_t command, uint8_t* buffer);
  writeSerial(bool success, uint8_t* payload, int elements);

  int test; 

};

#endif