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

ColorSensor::colorData ColorSensor::getRGBData() {
  curr_millis = millis();
  if ((curr_millis - prev_millis) > 100) {
    tcs.setInterrupt(false);  // turn on LED
    tcs.getRGB(&RGBData.red, &RGBData.green, &RGBData.blue);
    //tcs.setInterrupt(true);   // turn off LED
    // Store the RGB readings in the circular buffer
    rgbBuffer[bufferIndex++] = RGBData;
    if (bufferIndex >= BUFFER_SIZE) {
      bufferIndex = 0;
    }
    RGBData.red = int(RGBData.red);
    RGBData.green = int(RGBData.green);
    RGBData.blue = int(RGBData.blue);
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

    // Update the RGBData with the average values
    RGBData.red = int(avgRed);
    RGBData.green = int(avgGreen);
    RGBData.blue = int(avgBlue);


    // Update the previous millis
    prev_millis = curr_millis;
  }
  // Serial.print("R: "); Serial.print(int(RGBData.red));
  // Serial.print(" G: "); Serial.print(int(RGBData.green));
  // Serial.print(" B: "); Serial.print(int(RGBData.blue));
  // Serial.println();
  return RGBData;
}
