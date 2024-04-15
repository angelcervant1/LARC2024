#ifndef Gripper_h
#define Gripper_h

#include "Arduino.h"
#include <Stepper.h>
#include <Servo.h>

class Gripper {
private:
    static constexpr uint8_t stepperDigitalPin = 1; 
    static constexpr uint8_t stepperDirectionPin = 3; 
    static constexpr uint8_t servoRAnalogPin = 28;
    static constexpr uint8_t servoLAnalogPin = 30; //prev pin 27  
    static constexpr int stepsPerRevolution = 200;
    static constexpr int stepsPerLevel = 100; //example
    static constexpr int degreesToRelease = 115; //example 
    static constexpr int degreesToGrab = 70; //example 

public:
    Gripper(); // Constructor
    Stepper myStepper;
    Servo myServoR; 
    Servo myServoL; 
    void downLevel(const uint8_t level); 
    void upLevel(const uint8_t level); 
    void downSteps(const uint8_t steps); 
    void upSteps(const uint8_t steps); 
    void grabCube(); 
    void releaseCube(); 
    void stop(); 
};

#endif
