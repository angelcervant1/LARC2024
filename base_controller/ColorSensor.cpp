#include "ColorSensor.h"

ColorSensor::ColorSensor() {
  initColorSensor();
}

void ColorSensor::initColorSensor() {
  //Serial.begin(115200);
  if (tcs.begin()) {
  } else {
    Serial.println("No TCS34725 found ... check your connections");
  }
}

void ColorSensor::getRGBData(colorData &data) {
    curr_millis = millis();
    if ((curr_millis - prev_millis) > 50) {
        //tcs.setInterrupt(false);  // turn on LED
        tcs.getRGB(&data.red, &data.green, &data.blue);
        //tcs.setInterrupt(true);   // turn off LED
        // Store the RGB readings in the circular buffer
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

        // Update the data with the average values
        data.red = int(avgRed);
        data.green = int(avgGreen);
        data.blue = int(avgBlue);

        // Update the previous millis
        prev_millis = curr_millis;

     
    }
}

