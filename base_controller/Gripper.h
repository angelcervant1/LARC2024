#ifndef Gripper_h
#define Gripper_h

#include "Arduino.h"
#include "AccelStepper.h"
#include <Servo.h>

class Gripper {
private:
    static constexpr uint8_t stepperDigitalPin = 1; 
    static constexpr uint8_t stepperDirectionPin = 3; 
    static constexpr uint8_t servoRAnalogPin = 28;
    static constexpr uint8_t servoLAnalogPin = 30; //prev pin 27  
    static constexpr int stepsPerRevolution = 200;
    static constexpr int stepsPerLevel = 100; //example
    static constexpr int degreesToRelease = 100; //example 
    static constexpr int degreesToGrab = 75; //example
    static constexpr int limitSwitchPin = 31; //example
    int currentPosition = 0; // Initialize the current position 
    bool direction = HIGH;
    int Nema_Steep = 49;
    int Nema_Direction = 48;
    int Nivel= 0;
    int Paso= 4049; // pasos para llegar a otro nivel del estante
    int Ajuste=0;
    bool isHome = false, gripping, releasing;
    unsigned long long last_time, current_time, graspStartTime, releaseStartTime; 
    
public:
    Gripper(); // Constructor
    AccelStepper myStepper;
    Servo myServoR; 
    Servo myServoL; 
    void downLevel(const uint8_t level); 
    void upLevel(const uint8_t level); 
    void downSteps(int steps); 
    void upSteps(int steps); 
    void grabCube(); 
    void releaseCube(); 
    void stop(); 
    void StepperHome(); 
    void sequenceUp(unsigned long curr_millis); 
    void sequenceDown(unsigned long curr_millis); 


};

#endif