#ifndef Gripper_h
#define Gripper_h

#include "Arduino.h"
#include <Stepper.h>
#include <Servo.h>


class Gripper{

    private:
        static constexpr uint8_t stepperDigitalPin = 1;
        static constexpr uint8_t stepperDirectionPin  = 2;
        static constexpr uint8_t servosAnalogPins[3] = {31,0,0}; //Need to updated pins        
        Servo myServo;
    public:
        Gripper();
        void UpSteps(const uint8_t steps);
        void DownSteps(const uint8_t steps);
        void stop();
        void upLevel(const uint8_t level);
        void downLevel(const uint8_t level);
        void grabCube();

};

#endif
