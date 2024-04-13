#include "Movement.h"

//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement(BNO *bno, LineSensor *lineSensor) : kinematics_(kRPM, kWheelDiameter, kWheelBase, kWheelTrack,  bno)
{ 
  pidBno = PID(kBnoKP, kBnoKI, kBnoKD, 0, 180, kMaxErrorSum, 100);
  this->bno = bno;
  this->lineSensor = lineSensor;

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

  linear_x_ = 0; 
  linear_y_ = 0;
  angular_z_ = 0;
}

//////////////////////////////////PID//////////////////////////////////////
void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z){
  Kinematics::output rpm = kinematics_.getRPM(linear_x, linear_y, angular_z);
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
}

/*
void Movement::orientedMovement(const double linear_x, const double linear_y, double angular_z){
  bno->updateBNO();
  float current_angle = bno->getYaw();
  float angle_error = robotAngle_ - current_angle;
  
  if (current_angle >= 180){
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
  
    //Check if its shorter to turn clockwise or counterclockwise to reach the target angle
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
    //If the robot is already close to angle then stop 
    angular_z = 0.0;
    Serial.println("Already close to target angle");
    rpm = kinematics_.getRPM(linear_x, linear_y, angular_z); 
  }
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
}
*/
/*
void Movement::orientedMovement(const double linear_x, const double linear_y, double angular_z){
  bno->updateBNO();
  current_angle = bno->getYaw();
  float angle_error = robotAngle_ - current_angle;
  
 if (current_angle >= 180){
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
    angular_speed = (angular_speed * 2 * PI ) / 60;
    angular_speed = constrain(angular_speed, -kMaxAngularZ, kMaxAngularZ);
    Serial.println(angular_speed);

    if (current_angle <= robotAngle_) {
      angular_z = -angular_speed; 
      Serial.println("Turning counterclockwise");
    } else if (current_angle > robotAngle_) {
      angular_z = angular_speed; 
      Serial.println("Turning clockwise");
    }
    
    rpm = kinematics_.getRPM(0, 0, angular_z); 
  } else {
    angular_z = 0.0;
    //Serial.println("Already close to target angle");
    rpm = kinematics_.getRPM(linear_x, linear_y, angular_z); 
  }
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
}
*/


void Movement::orientedMovement(const double linear_x, const double linear_y, double angular_z){
  bno->updateBNO();
  current_angle = bno->getYaw();
  float angle_error;

  if (current_angle >= 180){
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

  // Adjust the proportional term to make angular speed proportional to the error
  float proportional_term = 0.03; // Adjust this value as needed

  if(abs(angle_error) > kAngleTolerance_){
    // Calculate the angular speed proportional to the error
    float angular_speed = abs(angle_error) * proportional_term;
    angular_speed = constrain(angular_speed, -kMaxAngularZ, kMaxAngularZ);
    Serial.println(angular_speed);
    if (current_angle <= robotAngle_) {
      angular_z = -angular_speed; 
      Serial.println("Turning counterclockwise");
    } else if (current_angle > robotAngle_) {
      angular_z = angular_speed; 
      Serial.println("Turning clockwise");
    }
    
    rpm = kinematics_.getRPM(0, 0, angular_z); 
  } else {
    angular_z = 0.0;
    //Serial.println("Already close to target angle");
    rpm = kinematics_.getRPM(linear_x, linear_y, angular_z); 
  }
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
}


//Used after localizing color pattern, to drive towards to it
void Movement::moveDirection(Direction direction, const double angleOffset){
    switch (direction){
        case FORWARD: //OK
          lineSensor->readDataFromSide(Right);
          sideDetected_[0] = lineSensor->lineDetectedFromSide();
          lineSensor->readDataFromSide(Left);
          sideDetected_[1] = lineSensor->lineDetectedFromSide();
          // Change linear velocity sign (+/-) if Angle offset is given
          linear_x_ = (robotAngle_ == 0) ? kMaxLinearX : kMaxLinearX;
          // Set linear_y_ based on side detection
          if(sideDetected_[0] == Right){
              linear_y_ = movementKp * kMaxLinearY;
          }
          else if(sideDetected_[1] == Left){
              linear_y_ = -movementKp * kMaxLinearY;
          }
          else{
            linear_y_ = 0;
          }
          Serial.println("FORWARD");
          break;
        case BACKWARD:
          lineSensor->readDataFromSide(Right);
          sideDetected_[0] = lineSensor->lineDetectedFromSide();
          lineSensor->readDataFromSide(Left);
          sideDetected_[1] = lineSensor->lineDetectedFromSide();
          linear_x_ = (robotAngle_ == (180)) ? -kMaxLinearX : -kMaxLinearX;
          // Set linear_y_ based on side detection
          if(sideDetected_[0] == Right){
              linear_y_ = movementKp * kMaxLinearY;
          }
          else if(sideDetected_[1] == Left){
              linear_y_ = -movementKp * kMaxLinearY;
          }
          else{
            linear_y_ = 0;
          }
          Serial.println("BACKWARD");
          break;
        case TOLEFT:
          lineSensor->readDataFromSide(Front);
          sideDetected_[2] = lineSensor->lineDetectedFromSide();
          lineSensor->readDataFromSide(Back);
          sideDetected_[3] = lineSensor->lineDetectedFromSide();
          linear_y_ = (robotAngle_ == 0) ? kMaxLinearY : kMaxLinearY;
          // Set linear_x_ based on side detection
          if(sideDetected_[2] == Front && sideDetected_[3] == None){
              linear_x_ = -movementKp * kMaxLinearX;
              Serial.println("BACKWARD");

          }
          else if(sideDetected_[3] == Back && sideDetected_[2] == None){
              linear_x_ = movementKp * kMaxLinearX;
              Serial.println("FORWARD");

          }
          else{
            linear_x_ = 0;
          }
          Serial.println("LEFT");
          break;
        case TORIGHT:
          lineSensor->readDataFromSide(Front);
          sideDetected_[2] = lineSensor->lineDetectedFromSide();
          lineSensor->readDataFromSide(Back);
          sideDetected_[3] = lineSensor->lineDetectedFromSide();
          linear_y_ = (robotAngle_ == 180) ? -kMaxLinearY : -kMaxLinearY;
          // Set linear_x_ based on side detection
          if(sideDetected_[2] == Front && sideDetected_[3] == None){
              linear_x_ = -movementKp * kMaxLinearX;
          }
          else if(sideDetected_[3] == Back && sideDetected_[2] == None){
              linear_x_ = movementKp * kMaxLinearX;
          }
          else{
              linear_x_ = 0;
          }
          Serial.println("RIGHT");
          break;
        default:
            stop(); 
            squaresCount = 0;
          break;
    }
  
  orientedMovement(linear_x_, linear_y_, angular_z_);
  
}


//When already at a known position, move given squares to a target
void Movement::moveDirection(Direction direction, const uint8_t squares, const double angleOffset, const double start_x_pos){
  lineSensor->readDataFromSide(Right);
  sideDetected_[0] = lineSensor->lineDetectedFromSide();
  lineSensor->readDataFromSide(Left);
  sideDetected_[1] = lineSensor->lineDetectedFromSide();
  lineSensor->readDataFromSide(Front);
  sideDetected_[2] = lineSensor->lineDetectedFromSide();
  lineSensor->readDataFromSide(Back);
  sideDetected_[3] = lineSensor->lineDetectedFromSide();
  
  globalDirection_ = direction;
  globalPosX_ = start_x_pos;

  if(squaresCount != squares){
    switch (direction){
        case FORWARD:
          //Change linear velocity sign (+/-) if Angle offset is given
          linear_x_ = (robotAngle_ == angleOffset) ? kMaxLinearX : kMaxLinearX;
          if(sideDetected_[0] == Right){
              linear_y_ = movementKp * kMaxLinearY;
          }
          else if(sideDetected_[1] == Left){
              linear_y_ = -movementKp * kMaxLinearY;
          }
          else{
            linear_y_ = 0;
          }
          if((sideDetected_[2] == Front) && firstLineDetected == false){
            firstLineDetected = true;
            prevSideDetected = sideDetected_[2];
          }
          else if(firstLineDetected && sideDetected_[3] == Back){
            if(prevSideDetected == Front){   
              squaresCount += 1;
              firstLineDetected = false;
            }
          }
          break;
        case BACKWARD:
          linear_x_ = (robotAngle_ == 180) ? -kMaxLinearX : -kMaxLinearX;
          if(sideDetected_[0] == Right){
              linear_y_ = movementKp * kMaxLinearY;
          }
          else if(sideDetected_[1] == Left){
              linear_y_ = -movementKp * kMaxLinearY;
          }
          else{
            linear_y_ = 0;
          }
          if((sideDetected_[2] == Front) && firstLineDetected == false){
            firstLineDetected = true;
            prevSideDetected = sideDetected_[2];
          }
          else if(firstLineDetected && sideDetected_[3] == Back){
            if(prevSideDetected == Front){   
              squaresCount += 1;
              firstLineDetected = false;
            }
          }
          break;
        case TOLEFT:
          linear_y_ = (robotAngle_ == angleOffset) ? kMaxLinearY : kMaxLinearY;
          if(sideDetected_[2] == Front  && sideDetected_[3] == None){
              linear_x_ = -movementKp * kMaxLinearX;
              Serial.println("BACKWARD");

          }
          else if(sideDetected_[3] == Back  && sideDetected_[2] == None){
              linear_x_ = movementKp * kMaxLinearX;
              Serial.println("FORWARD");

          }
          else{
            linear_x_ = 0;
          }
          if((sideDetected_[1] == Left) && firstLineDetected == false){
            firstLineDetected = true;
            prevSideDetected = sideDetected_[1];
            Serial.print("Side Detected: "); Serial.println(prevSideDetected);
          }
          else if(firstLineDetected  && sideDetected_[0] == Right){
              squaresCount += 1;
              Serial.println("Moved 1 square");
              firstLineDetected = false;
                globalPosX_ = (robotAngle_ == angleOffset) ? (globalPosX_ - squaresCount) : (globalPosX_ + squaresCount);
                if(globalPosX_ < 0)
                  globalPosX_ = 0;
                if(globalPosX_ > 7)
                  globalPosX_ = 7;
          }
          break;
          Serial.println("LEFT");

        case TORIGHT:
          linear_y_ = (robotAngle_ == 180) ? -kMaxLinearY : -kMaxLinearY;
          if(sideDetected_[2] == Front && sideDetected_[3] == 4){
              linear_x_ = -movementKp * kMaxLinearX;
            }
          else if(sideDetected_[3] == Back && sideDetected_[2] == 4){
              linear_x_ = movementKp * kMaxLinearX;
          }
          else{
            linear_x_ = 0;
          }
          if((sideDetected_[0] == Right) && firstLineDetected == false){
            firstLineDetected = true;
            prevSideDetected = sideDetected_[0];
          }
          else if(firstLineDetected == true && sideDetected_[1] == Left){
            if(prevSideDetected == Right){   
              squaresCount += 1;
              firstLineDetected = false;
                globalPosX_ = (robotAngle_ == angleOffset) ? (globalPosX_ + squaresCount) : (globalPosX_ - squaresCount);
                if(globalPosX_ < 0)
                  globalPosX_ = 0;
                if(globalPosX_ > 7)
                  globalPosX_ = 7;
            }
          }
          Serial.println("RIGHT");

          break;
        default:
            stop(); 
            squaresCount = 0;
          break;
      }
  }   
  else{
      //stop if robot has reached squares amount target
      stop();
      //squaresCount = 0;
  }
  Serial.print("Square Count: "); Serial.println(squaresCount);
  orientedMovement(linear_x_, linear_y_, angular_z_);
}

void Movement::driveToTarget(float coord_x, Direction direction){

  
}

void Movement::driveToTarget(float coord_x, const double linear_x, const double linear_y, const double angular_z){

}

void Movement::GoToSquare(){

}


void Movement::updatePIDKinematics(double fl_speed, double fr_speed, double bl_speed, double br_speed) {
  front_left_motor_.stableRPM(fl_speed);
  front_right_motor_.stableRPM(fr_speed);
  back_left_motor_.stableRPM(bl_speed);
  back_right_motor_.stableRPM(br_speed);
}

uint8_t Movement::getCurrentPosX(){
  return globalPosX_;
}


Direction Movement::getDirectionState(){
  return globalDirection_;
}

uint8_t Movement::getSquareCounter(){
  return squaresCount;
}

uint8_t Movement::setGlobalPosX(uint8_t globalPosX){
  globalPosX_ = globalPosX; 
}

uint8_t Movement::setSquareCounter(uint8_t squares){
  squaresCount = squares;
}