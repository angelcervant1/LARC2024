#ifndef ColorSensor_h
#define ColorSensor_h

#define commonAnode true

#include "Adafruit_TCS34725.h"


class ColorSensor{


    private:    
        Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
        
    public:
        static constexpr float kRedTreshold = 0;
        static constexpr float kGreenTreshold = 0;
        static constexpr float kBlueTreshold = 0;
        static constexpr float kYellowTreshold = 0;

        struct colorData{
            float red;
            float green;
            float blue;
        };

        ColorSensor();
        colorData getRGBData();
        void initColorSensor();
        float curr_millis, prev_millis;
        colorData RGBData; 
};

#endif