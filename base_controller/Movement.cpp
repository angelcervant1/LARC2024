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

void Movement::orientedMovement(const double linear_x, const double linear_y, double angular_z){
  bno->updateBNO();
  float current_angle = bno->getYaw();
  float angle_error = robotAngle_ - current_angle;
  
  if (current_angle > 180){
    current_angle -= 360;
    angle_error = robotAngle_ + current_angle;
  } else {
    angle_error = robotAngle_ - current_angle;
  }

  Serial.print("Setpoint:"); Serial.print(robotAngle_); 
  Serial.print(" "); 
  Serial.print("Current:"); Serial.println(current_angle);
  Serial.print("Angle Error: "); Serial.println(angle_error);
  
  Kinematics::output rpm;
  
  if(abs(angle_error) > kAngleTolerance_){
    float angular_speed = pidBno.compute_dt(robotAngle_, abs(current_angle), kBNO_time);
    float max_angular_change = 1.2; 
    angular_speed = constrain(angular_speed, -max_angular_change, max_angular_change);
    
    
    //Check if it's shorter to turn clockwise or counterclockwise to reach the target angle
    if (current_angle < robotAngle_) {
      angular_z = -angular_speed; 
      Serial.println("Turning counterclockwise");
    } else if (current_angle > robotAngle_) {
      angular_z = angular_speed; 
      Serial.println("Turning clockwise");
    }
    //Serial.print("Angular Speed: "); Serial.println(angular_speed);
    rpm = kinematics_.getRPM(0, 0, angular_z); 
    
  } else {
    //If the robot is already close to the target angle, stop turning
    angular_z = 0.0;
    Serial.println("Already close to target angle");
    rpm = kinematics_.getRPM(linear_x, linear_y, angular_z); 
  }
  
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
}

void Movement::updatePIDKinematics(double fl_speed, double fr_speed, double bl_speed, double br_speed) {
  front_left_motor_.stableRPM(fl_speed);
  front_right_motor_.stableRPM(fr_speed);
  back_left_motor_.stableRPM(bl_speed);
  back_right_motor_.stableRPM(br_speed);
}
