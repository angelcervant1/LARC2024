#ifndef RASPY_h
#define RASPY_h

#include "Movement.h"
#include "Arduino.h"
class Raspy{
  public: 
    Raspy();
    void executeCommand(uint8_t packet_size, uint8_t command, uint8_t* buffer);
    void writeSerial(bool success, uint8_t* payload, int elements);
    void readSerial();
    String flag;
    int tile;
    int color;
    uint8_t cube_offset;
    String get_status();
    int get_tile();
    int get_color();
  private:
    BNO *_bno;
    LineSensor *_line;
    ColorSensor *_color;
    Movement *_robot;
    Gripper *_gripper;
};

#endif
