#include "ColorSensor.h"

ColorSensor::ColorSensor(){
  initColorSensor();
}

void ColorSensor::initColorSensor(){
  if (tcs.begin()) {
  } else {
    Serial.println("No TCS34725 found ... check your connections");
  }
  
}
    
colorData ColorSensor::getRGBData() {
    curr_millis = millis();
    if((curr_millis - prev_millis) > 60){
        tcs.setInterrupt(false);  // turn on LED
        tcs.getRGB(&RGBData.red, &RGBData.green, &RGBData.blue);
        tcs.setInterrupt(true);  // turn off LED

         Serial.print("R:\t"); Serial.print(int(RGBData.red));
         Serial.print("\tG:\t"); Serial.print(int(RGBData.green));
         Serial.print("\tB:\t"); Serial.print(int(RGBData.blue));
        
        Serial.print("\n");
        prev_millis = curr_millis;

    }
    return RGBData;
   
}
