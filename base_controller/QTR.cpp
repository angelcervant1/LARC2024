#include "QTR.h"

LineSensor::LineSensor() {
    Serial.begin(115200);
    pinMode(muxD1, OUTPUT);
    pinMode(muxD2, OUTPUT);
    pinMode(muxD3, OUTPUT);
    pinMode(muxD4, OUTPUT); 
    pinMode(lineDataPin, INPUT);
}


void LineSensor::readDataFromSide(SignalSide side) {
    switch(side) {
        case Front:
            strSide = "Front";
            break;
        case Left:
            strSide = "Left";
            break;
        case Back:
            strSide = "Back";
            break;
        case Right:
            strSide = "Right";
            break;
    }

    side_ = side;

    for(int sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
        digitalWrite(muxD4, combinations[side][sensorIndex][0]);
        digitalWrite(muxD3, combinations[side][sensorIndex][1]);
        digitalWrite(muxD2, combinations[side][sensorIndex][2]);
        digitalWrite(muxD1, combinations[side][sensorIndex][3]);
        sensorData[sensorIndex] = analogRead(lineDataPin);
        //  Serial.print("//////\nData from side: ");
        //  Serial.print(strSide);
        //  Serial.print(", Sensor ");
        //  Serial.print(sensorIndex + 1);
        //  Serial.print(": ");
        //  Serial.println(sensorData[sensorIndex]);
    }   
        //sideDetected_ = lineDetectedFromSide();
        //Serial.print("Line Detected from: "); Serial.println(sideDetected_);

}

SignalSide LineSensor::lineDetectedFromSide() {
    for(int i = 0; i < 4; i++) {
        if(sensorData[i] > threshold) {
            return side_;
        }
    }
    return None;
}

void LineSensor::readAllData() {
    readDataFromSide(Front);
    readDataFromSide(Left);
    readDataFromSide(Back);
    readDataFromSide(Right);
}
