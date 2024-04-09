#include "BNO.h"

//////////////////////////////////Constructor//////////////////////////////////////
BNO::BNO() {
  bno_ = Adafruit_BNO055(55);
  if (!bno_.begin()) {
    Serial.println("Oops no bno detected..");
    return;
  }
  Serial.begin(115200);
  bno_.setExtCrystalUse(true);
  sensors_event_t event;
  bno_.getEvent(&event);
}

//////////////////////////////////Calibration//////////////////////////////////////
uint8_t BNO::orientationStatus() {
  uint8_t system, gyro, accel, mag = 0;
  bno_.getCalibration(&system, &gyro, &accel, &mag);

  return mag;
}

void BNO::updateBNO() {
  sensors_event_t orientationData;
  bno_.getEvent(&orientationData);
  yaw_ = orientationData.orientation.x;
  }


float BNO::getYaw() {
  return yaw_;
}
float BNO::getYawVel() {
  return yaw_vel_;
}
float BNO::getXAccel() {
  return x_accel;
}
float BNO::getYAccel() {
  return y_accel;
}
float BNO::getZAccel() {
  return z_accel;
}

//////////////////////////////////Get Functions//////////////////////////////////////
void BNO::reset() {
  digitalWrite(reset_pin_, LOW);  
  delayMicroseconds(30);
  digitalWrite(reset_pin_, HIGH);
  bno_ = Adafruit_BNO055(55);
}
