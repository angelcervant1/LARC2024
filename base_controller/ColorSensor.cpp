#include "ColorSensor.h"

ColorSensor::ColorSensor() {
  initColorSensor();
}

void ColorSensor::initColorSensor() {
  //Serial.begin(115200);
  if (tcs.begin()) {
  } else {
    //serial.println("No TCS34725 found ... check your connections");
  }
}

void ColorSensor::getRGBData(colorData &data) {
    curr_millis = millis();
    if ((curr_millis - prev_millis) > 50) {
        //tcs.setInterrupt(false);  // turn on LED
        tcs.getRGB(&data.red, &data.green, &data.blue);
        //tcs.setInterrupt(true);   // turn off LED
        rgbBuffer[bufferIndex++] = data;
        if (bufferIndex >= BUFFER_SIZE) {
            bufferIndex = 0;
        }
        data.red = int(data.red);
        data.green = int(data.green);
        data.blue = int(data.blue);
        // Calculate the average RGB values
        int avgRed = 0, avgGreen = 0, avgBlue = 0;
        for (int i = 0; i < BUFFER_SIZE; i++) {
            avgRed += rgbBuffer[i].red;
            avgGreen += rgbBuffer[i].green;
            avgBlue += rgbBuffer[i].blue;
        }
        avgRed /= BUFFER_SIZE;
        avgGreen /= BUFFER_SIZE;
        avgBlue /= BUFFER_SIZE;

        data.red = uint8_t(avgRed);
        data.green = uint8_t(avgGreen);
        data.blue = uint8_t(avgBlue);
        
        // Update the previous millis
        prev_millis = curr_millis;
        // Serial.println(" R: "); Serial.print(data.red);
        // Serial.print(" G: "); Serial.print(data.green);
        // Serial.print(" B: "); Serial.print(data.blue);
        // Serial.println();
     
    }
}