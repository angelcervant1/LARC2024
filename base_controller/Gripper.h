#ifndef Gripper_h
#define Gripper_h

#include "Arduino.h"

class Gripper{

    private:
        static constexpr uint8_t stepperDigitalPin = 1;
        static constexpr uint8_t stepperDirectionPin  = 2;
        static constexpr uint8_t servosAnalogPins[3] = {1,3,4}; //Need to updated pins
        
    public:
        void UpSteps(const uint8_t steps);
        void DownSteps(const uint8_t steps);
};

#endif
