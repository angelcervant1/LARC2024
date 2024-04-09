#include "Movement.h"

//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement(BNO *bno) : kinematics_(kRPM, kWheelDiameter, kWheelBase, kWheelTrack,  bno)
{ 
  pidBno = PID(kBnoKP, kBnoKI, kBnoKD, 0, 180, kMaxErrorSum, 100);
  this->bno = bno;
  back_left_motor_ = Motor(MotorId::BackLeft, kDigitalPinsBackLeftMotor[1], 
                          kDigitalPinsBackLeftMotor[0], kAnalogPinBackLeftMotor, 
                          kEncoderPinsBackLeftMotor);
  front_left_motor_ = Motor(MotorId::FrontLeft, kDigitalPinsFrontLeftMotor[1], 
                            kDigitalPinsFrontLeftMotor[0], kAnalogPinFrontLeftMotor, 
                            kEncoderPinsFrontLeftMotor);
  back_right_motor_ = Motor(MotorId::BackRight, kDigitalPinsBackRightMotor[0], 
                            kDigitalPinsBackRightMotor[1], kAnalogPinBackRightMotor, 
                            kEncoderPinsBackRightMotor);
  front_right_motor_ = Motor(MotorId::FrontRight, kDigitalPinsFrontRightMotor[0], 
                            kDigitalPinsFrontRightMotor[1], kAnalogPinFrontRightMotor, 
                            kEncoderPinsFrontRightMotor);
}

//////////////////////////////////Encoders//////////////////////////////////////
void Movement::initEncoders() {
  back_left_motor_.initEncoders();
  front_left_motor_.initEncoders();
  back_right_motor_.initEncoders();
  front_right_motor_.initEncoders();
}

////////////////////////////SET A DESIRED ROBOT ANGLE//////////////////////////////
void Movement::setRobotAngle(const double angle){
  robotAngle_ = angle;
}

//////////////////////////////////PWM//////////////////////////////////////
void Movement::changePwm(const uint8_t pwm) {
  back_left_motor_.changePwm(pwm);
  front_left_motor_.changePwm(pwm);
  back_right_motor_.changePwm(pwm);
  front_right_motor_.changePwm(pwm);
}

//////////////////////////////////VELOCITY//////////////////////////////////////
void Movement::setDeltaX(const double delta_x) {
  delta_x_ = delta_x;
}

void Movement::setDeltaY(const double delta_y) {
  delta_y_ = delta_y;
}

void Movement::setDeltaAngular(const double delta_angular) {
  delta_angular_ = delta_angular;
}

double Movement::getDeltaX(){
  return delta_x_;
}

double Movement::getDeltaY(){
  return delta_y_;
}

double Movement::getDeltaAngular(){
  return delta_angular_;
}

double Movement::radiansToDegrees(const double radians) {
  return radians * 180 / M_PI;
}

void Movement::stop() {
  back_left_motor_.stop();
  front_left_motor_.stop();
  back_right_motor_.stop();
  front_right_motor_.stop();
}

//Returns the constrained velocity of the robot.
double constrainDa(double x, double min_, double max_)
{
  return max(min_, min(x, max_));
}

//////////////////////////////////PID//////////////////////////////////////
void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z){
  
  Kinematics::output rpm = kinematics_.getRPM(linear_x, linear_y, angular_z);
  
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);

}

//////////////////////////ADJUSTING CMD VELOCITY BASED ON BNO FEEDBACK//////////////////////////
void Movement::orientedMovement(const double linear_x, const double linear_y, double angular_z){
  bno->updateBNO();
  float current_angle = bno->getYaw();
  float angle_error = robotAngle_ - current_angle;
  if (current_angle>180){
    current_angle-=360;
    angle_error = robotAngle_ + current_angle;
  } else {
    angle_error = robotAngle_ - current_angle;
  }

  Serial.print("Setpoint:");Serial.print(robotAngle_); 
  Serial.print(" "); 
  Serial.print("Current:"); Serial.println(current_angle);
  Kinematics::output rpm;
  
  if(abs(angle_error) > kAngleTolerance_){
    float angular_speed = pidBno.compute_dt(robotAngle_, abs(current_angle), kBNO_time);
    float max_angular_change = 1.0; 
    angular_speed = constrain(angular_speed, -max_angular_change, max_angular_change);
    
    //Check if its shorter to turn clockwise or counterclockwise to reach the target angle
    
    if (current_angle < robotAngle_) {
      angular_z = -angular_speed; 
    } else if (current_angle > robotAngle_){
      angular_z = angular_speed; 
    }
    rpm = kinematics_.getRPM(0, 0, angular_z); 
    
  } else {
    //If the robot is already close to the target angle, stop turning
    angular_z = 0.0;
    rpm = kinematics_.getRPM(linear_x, linear_y, angular_z); 
  
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);

  //Serial.println("//////////////////");
  //Serial.print("Goal Angle:"); Serial.println(robotAngle_);
  //Serial.print("Angle Error:"); Serial.println(angle_error);
  //Serial.print("Current Angle:"); Serial.println(current_angle);
  //Serial.print("Angular Speed:"); Serial.println(angular_z);
  //Serial.println(" ");
  //Serial.println("//////////////////");
  //Serial.flush();
}


/*
void Movement::orientedMovement(const double linear_x, const double linear_y, double angular_z){
  bno->updateBNO();
  float current_angle = bno->getYaw();
  float angle_error = robotAngle_ - current_angle;
  Kinematics::output rpm;

  if(abs(angle_error) > kAngleTolerance_){
    // Calculate the remaining angle difference
    float remaining_angle_difference = abs(angle_error);
    
    // Define a base angular speed
    float base_angular_speed = 0.4; // Adjust as needed
    
    // Calculate the proportional angular speed based on remaining angle difference
    float proportional_angular_speed = base_angular_speed * (remaining_angle_difference / 180.0); // Assuming angle in degrees
    
    // Adjust the sign of angular speed based on the desired angle (e.g., 180 degrees)
    float target_angle = robotAngle_; // Adjust as needed
    if (angle_error > 0 && current_angle < target_angle) {
      // If the current angle is less than the target angle, turn clockwise
      angular_z = proportional_angular_speed;
    } else if (angle_error < 0 && current_angle > target_angle) {
      // If the current angle is greater than the target angle, turn counterclockwise
      angular_z = -proportional_angular_speed;
    } else {
      // If the robot is already close to the target angle, stop turning
      angular_z = 0.0;
    }

    rpm = kinematics_.getRPM(0, 0, angular_z);
  }
  else{
    rpm = kinematics_.getRPM(linear_x, linear_y, angular_z);
  }
  
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);

  Serial.println(" ");
  Serial.println("//////////////////");
  Serial.print("Goal Angle:"); Serial.println(robotAngle_);
  Serial.print("Angle Error:"); Serial.println(angle_error);
  Serial.print("Current Angle:"); Serial.println(current_angle);
  Serial.print("Angular Speed:"); Serial.println(angular_z);
  Serial.println(" ");
  Serial.println("//////////////////");
}
*/

void Movement::updatePIDKinematics(double fl_speed, double fr_speed, double bl_speed, double br_speed) {
  front_left_motor_.stableRPM(fl_speed);
  front_right_motor_.stableRPM(fr_speed);
  back_left_motor_.stableRPM(bl_speed);
  back_right_motor_.stableRPM(br_speed);
}
