#ifndef ColorSensor_h
#define ColorSensor_h

#define commonAnode true
#define BUFFER_SIZE 4

#include "Adafruit_TCS34725.h"


class ColorSensor{


    private:    
        Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
        
    public:
        static constexpr uint8_t kRedTreshold = 195;
        static constexpr uint8_t kGreenTreshold = 103;
        static constexpr uint8_t kBlueTreshold = 102;
        static constexpr uint8_t kYellowTreshold_R = 123;
        static constexpr uint8_t kYellowTreshold_G = 79;
        static constexpr uint8_t kYellowTreshold_B = 40;
        uint8_t bufferIndex = 0; // Initialize bufferIndex


        struct colorData{
            float red;
            float green;
            float blue;
        }rgbBuffer[BUFFER_SIZE];;

        ColorSensor();
        void getRGBData(colorData &data); // Updated function signature        
        void initColorSensor();
        float curr_millis = 0, prev_millis = 0;
};

#endif
