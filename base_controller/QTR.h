#ifndef QTR_h
#define QTR_h

#include <Arduino.h>

enum SignalSide {
    Left, //QTR1 
    Right, //QTR22
    Back, //QTR3 
    Front, //QTR4
    None //5
};

class LineSensor {
private:
    static constexpr uint8_t muxD1 = 54; 
    static constexpr uint8_t muxD2 = 55; 
    static constexpr uint8_t muxD3 = 56; 
    static constexpr uint8_t muxD4 = 57; 
    static constexpr uint8_t lineDataPin = 58; 

    int combinations[4][4][4] = {
        {
            // Left side combinations
            {0, 0, 0, 0}, // C1
            {0, 0, 0, 1}, // C2
            {0, 0, 1, 0}, // C3
            {0, 0, 1, 1}  // C4
        },
        {
            // Right side combinations
            {0, 1, 0, 0}, // C5
            {0, 1, 0, 1}, // C6
            {0, 1, 1, 0}, // C7
            {0, 1, 1, 1}  // C8
        },
        {
            // Back side combinations
            {1, 0, 0, 0}, // C9
            {1, 0, 0, 1}, // C10
            {1, 0, 1, 0}, // C11
            {1, 0, 1, 1}  // C12
        },
        {
            // Front side combinations
            {1, 1, 0, 0}, // C13
            {1, 1, 0, 1}, // C14
            {1, 1, 1, 0}, // C15
            {1, 1, 1, 1}  // C16
        }

    };

    int sensorData[4];
    int threshold = 800; 
    String strSide;
    SignalSide sideDetected_, side_;
public:
    LineSensor();
    void readDataFromSide(SignalSide side);
    void readAllData();
    SignalSide lineDetectedFromSide();
};

#endif
