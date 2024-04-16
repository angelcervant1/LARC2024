#ifndef ColorSensor_h
#define ColorSensor_h

#define commonAnode true
#define BUFFER_SIZE 4  

#include "Adafruit_TCS34725.h"


class ColorSensor{


    private:    
        Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
        
    public:
        static constexpr uint8_t kRedTreshold = 170;
        static constexpr uint8_t kGreenTreshold = 108;
        static constexpr uint8_t kBlueTreshold = 95;
        static constexpr uint8_t kYellowTreshold_R = 120;
        static constexpr uint8_t kYellowTreshold_G = 78;
        static constexpr uint8_t kYellowTreshold_B = 0;
        uint8_t bufferIndex = 0; // Initialize bufferIndex


        struct colorData{
            float red;
            float green;
            float blue;
        }rgbBuffer[BUFFER_SIZE];;

        ColorSensor();
        colorData getRGBData();
        void initColorSensor();
        float curr_millis = 0, prev_millis = 0;
        colorData RGBData; 
};

#endif