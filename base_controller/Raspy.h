#ifndef RASPY_h
#define RASPY_h

#include "Movement.h"
#include "Arduino.h"
class Raspy{
  public: 
    Raspy(BNO *bno, LineSensor *line, ColorSensor *color, Movement *robot, Gripper *gripper);
    void executeCommand(uint8_t packet_size, uint8_t command, uint8_t* buffer);
    void writeSerial(bool success, uint8_t* payload, int elements);
    void readSerial();
    String flag;
    uint8_t cube_offset;
  
  private:
    BNO *_bno;
    LineSensor *_line;
    ColorSensor *_color;
    Movement *_robot;
    Gripper *_gripper;
};

#endif
