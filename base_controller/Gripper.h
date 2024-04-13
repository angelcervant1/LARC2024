#ifndef Gripper_h
#define Gripper_h

#include "Arduino.h"
#include <Stepper.h>
#include <Servo.h>

class Gripper {
private:
    static constexpr uint8_t stepperDigitalPin = 1; 
    static constexpr uint8_t stepperDirectionPin = 3; 
    static constexpr uint8_t servoAnalogPin = 2; 
    static constexpr int stepsPerRevolution = 200; 
    static constexpr int stepsPerLevel = 300; //example 
    Stepper myStepper{stepsPerRevolution, stepperDigitalPin, stepperDirectionPin}; 
    Servo myServo; 

public:
    Gripper(); // Constructor
    void downLevel(const uint8_t level); 
    void upLevel(const uint8_t level); 
    void downSteps(const uint8_t steps); 
    void upSteps(const uint8_t steps); 
    void grabCube(); 
    void stop(); 
};

#endif
