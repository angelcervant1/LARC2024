#include "Movement.h"

//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement(BNO *bno, LineSensor *lineSensor, ColorSensor *colorSensor) : kinematics_(kRPM, kWheelDiameter, kWheelBase, kWheelTrack)
{ 
  pidBno = PID(kBnoKP, kBnoKI, kBnoKD, 0, 180, kMaxErrorSum, 100);
  this->bno = bno;
  this->lineSensor = lineSensor;
  this->colorSensor = colorSensor;

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
  this->past_check = millis();
  this->detect_tile = false;
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

void Movement::setInitialRobotAngle(double angle){
  originAngle = angle;
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


// void Movement::orientedMovement(const double linear_x, const double linear_y, double angular_z){
//   bno->updateBNO();
//   current_angle = bno->getYaw();
//   float angle_error;

//   if (current_angle >= 180){
//     current_angle -= 360;
//     angle_error = robotAngle_ + current_angle;
//   } else {
//     angle_error = robotAngle_ - current_angle;
//   }

//  Serial.print("Setpoint:"); Serial.print(robotAngle_); 
//  Serial.print(" "); 
//  Serial.print("Current:"); Serial.println(current_angle);
//  Serial.print("Angle Error: "); Serial.println(angle_error);
  
//   Kinematics::output rpm;

//   // Adjust the proportional term to make angular speed proportional to the error
//   float proportional_term = 0.01; // Adjust this value as needed

//   if(abs(angle_error) > kAngleTolerance_){
//     // Calculate the angular speed proportional to the error
//     float angular_speed = abs(angle_error) * proportional_term;
//     angular_speed = constrain(angular_speed, -kMaxAngularZ, kMaxAngularZ);
//     Serial.println(angular_speed);
//     if (current_angle > 0) {
//       angular_z = -angular_speed; 
//       Serial.println("Turning counterclockwise");
//     } else if (current_angle < 0) {
//       angular_z = angular_speed; 
//       Serial.println("Turning clockwise");
//     }
    
//     rpm = kinematics_.getRPM(0, 0, angular_z); 
//   } else {
//     angular_z = 0.0;
//     angleOffsetReached = true;
//     //Serial.println("Already close to target angle");
//     rpm = kinematics_.getRPM(linear_x, linear_y, angular_z); 
//   }
//   updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
// }

void Movement::orientedMovement(const double linear_x, const double linear_y, double angular_z){
  bno->updateBNO();
  current_angle = bno->getYaw();
  float angle_error;

  // Ensure current angle is in the range [0, 360]
  if (current_angle < 0) {
    current_angle += 360;
  }

  // Calculate the smallest angle difference to the target angle
  angle_error = fmod(robotAngle_ - current_angle + 360, 360);
  if (angle_error > 180) {
    angle_error -= 360;
  }

  // Serial.print("Setpoint: "); Serial.print(robotAngle_); 
  // Serial.print(" Current: "); Serial.println(current_angle);
  // Serial.print("Angle Error: "); Serial.println(angle_error);
  
  Kinematics::output rpm;

  // Adjust the proportional term to make angular speed proportional to the error
  float proportional_term = 0.1; // Adjust this value as needed

  if (fabs(angle_error) > kAngleTolerance_){
    // Calculate the angular speed proportional to the error
    float angular_speed = fabs(angle_error) * proportional_term;
    angular_speed = constrain(angular_speed, -kMaxAngularZ, kMaxAngularZ);

    // Determine the direction of rotation
    if (angle_error > 0) {
      angular_z = -angular_speed; // Rotate clockwise
      Serial.println("Turning clockwise");
    } else {
      angular_z = angular_speed; // Rotate counterclockwise
      Serial.println("Turning counterclockwise");
    }
    
    rpm = kinematics_.getRPM(0, 0, angular_z); 
  } else {
    angular_z = 0.0;
    angleOffsetReached = true;
    //Serial.println("Already close to target angle");
    rpm = kinematics_.getRPM(linear_x, linear_y, angular_z); 
  }
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
}


//Used after localizing color pattern, to drive towards to it
void Movement::moveDirection(Direction direction, const  double angleOffset){
    setRobotAngle(angleOffset);
    switch (direction){
        case FORWARD: //OK
          lineSensor->readDataFromSide(Right);
          sideDetected_[0] = lineSensor->lineDetectedFromSide();
          lineSensor->readDataFromSide(Left);
          sideDetected_[1] = lineSensor->lineDetectedFromSide();
          // Change linear velocity sign (+/-) if Angle offset is given
          linear_x_ = (robotAngle_ == angleOffset) ? kMaxLinearX : kMaxLinearX;
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
          linear_x_ = (robotAngle_ == (angleOffset - 180)) ? -kMaxLinearX : -kMaxLinearX;
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
          linear_y_ = (robotAngle_ == (angleOffset - 180)) ? kMaxLinearY : kMaxLinearY;
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
          linear_y_ = (robotAngle_ == angleOffset) ? -kMaxLinearY : -kMaxLinearY;
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

void Movement::moveDirection(Direction direction, const uint8_t squares, const double angleOffset) {
    // Read data from sensors
    lineSensor->readDataFromSide(Right);
    sideDetected_[0] = lineSensor->lineDetectedFromSide();
    lineSensor->readDataFromSide(Left);
    sideDetected_[1] = lineSensor->lineDetectedFromSide();
    lineSensor->readDataFromSide(Front);
    sideDetected_[2] = lineSensor->lineDetectedFromSide();
    lineSensor->readDataFromSide(Back);
    sideDetected_[3] = lineSensor->lineDetectedFromSide();
  
    // Set the global movement direction
    //globalDirection_ = direction;

    // Determine the sign for adjusting globalPosX based on the robot's angle offset
    int posXChange;    // if (originAngle == angleOffset) {
    //     posXAdjustment = 1; // Increment globalPosX
    // } else if (originAngle == (angleOffset + 180)) {
    //     posXAdjustment = -1; // Decrement globalPosX
    // }
    // Update linear_x_ and linear_y_ based on the movement direction and the robot's angle offset
    if(!(squaresCount == squares)){
        switch (direction) {
          case FORWARD:
              linear_x_ = -kMaxLinearX;
              if(sideDetected_[0] == Right){
                  linear_y_ = movementKp * kMaxLinearY;
                  Serial.println("LEFT");

              }
              else if(sideDetected_[1] == Left){
                  linear_y_ = -movementKp * kMaxLinearY;
                  Serial.println("RGHT");

              }
              else{
                linear_y_ = 0;
                Serial.println("FORWARD");

              }
              posXChange = (originAngle == angleOffset) ? -1 : 1;
              // Update globalPosX if the robot is moving left and a line is detected on the left side
              if (sideDetected_[2] == Front && !firstLineDetected) {
                  firstLineDetected = true;
              } else if (firstLineDetected && sideDetected_[3] == Back) {
                  globalPosY_ += posXChange;
                  Serial.print("Moved Forward.  ");
                  Serial.print("Global Pos Y: "); 
                  Serial.println(globalPosY_);
                  firstLineDetected = false;
              }
          case BACKWARD:
            linear_x_ = kMaxLinearX;
            if(sideDetected_[0] == Right){
                linear_y_ = movementKp * kMaxLinearY;
            }
            else if(sideDetected_[1] == Left){
                linear_y_ = -movementKp * kMaxLinearY;
            }
            else{
              linear_y_ = 0;
              Serial.println("BACKWARD");

            }
              posXChange = (originAngle == angleOffset) ? 1 : -1;

            if (sideDetected_[3] == Back && !firstLineDetected) {
                  firstLineDetected = true;
              } else if (firstLineDetected && sideDetected_[2] == Front) {
                  globalPosY_ += posXChange;
                  Serial.print("Moved Forward.  ");
                  Serial.print("Global Pos Y: "); 
                  Serial.println(globalPosY_);
                  firstLineDetected = false;
              }           
             break;
          case TOLEFT:
            linear_y_ = kMaxLinearY;
            if(sideDetected_[2] == Front  && sideDetected_[3] == None){
                linear_x_ = -movementKp * kMaxLinearX;
                Serial.println("BACKWARD");
            }
            else if(sideDetected_[3] == Back  && sideDetected_[2] == None){
                linear_x_ = movementKp * kMaxLinearX;
                Serial.println("FORWARD");
            }
            else
              linear_x_ = 0;

              Serial.println("LEFT");              // Calculate the change in position based on the movement direction
              posXChange = (originAngle == angleOffset) ? -1 : 1;
              // Update globalPosX if the robot is moving left and a line is detected on the left side
              if (sideDetected_[1] == Left && !firstLineDetected) {
                  firstLineDetected = true;
              } else if (firstLineDetected && sideDetected_[0] == Right) {
                  globalPosX_ += posXChange;
                  Serial.print("Moved left.  ");
                  Serial.print("Global Pos X: "); 
                  Serial.println(globalPosX_);
                  firstLineDetected = false;
              }
              Serial.println("LEFT");
              break;

          case TORIGHT:
            linear_y_ = (robotAngle_ == (angleOffset - 180)) ? -kMaxLinearY : -kMaxLinearY;
            if(sideDetected_[2] == Front && sideDetected_[3] == None){
                linear_x_ = -movementKp * kMaxLinearX;
              }
            else if(sideDetected_[3] == Back && sideDetected_[2] == None){
                linear_x_ = movementKp * kMaxLinearX;
            }
            else{
              linear_x_ = 0;
            }              // Calculate the change in position based on the movement direction
              posXChange = (originAngle == angleOffset) ? 1 : -1;

              // Update globalPosX if the robot is moving right and a line is detected on the right side
            if (sideDetected_[0] == Right && !firstLineDetected) {
                  firstLineDetected = true;
              } else if (firstLineDetected && sideDetected_[1] == Left) {
                  globalPosX_ += posXChange;
                  Serial.print("Moved left.  ");
                  Serial.print("Global Pos X: "); 
                  Serial.println(globalPosX_);
                  firstLineDetected = false;
              }
              Serial.println("RIGHT");
              break;
        }
    }
    
    orientedMovement(linear_x_, linear_y_, angular_z_);

}


void Movement::driveToColor(const double start_x_pos, Direction direction, colorNum color_id){
      
      colorSensor->getRGBData(rgbData);
      
      Serial.println(" R: "); Serial.print(rgbData.red);
      Serial.print(" G: "); Serial.print(rgbData.green);
      Serial.print(" B: "); Serial.print(rgbData.blue);
      Serial.println();

    bool shouldMoveBackward = false;
      switch (color_id) {
          case 0:
              shouldMoveBackward = rgbData.red > ColorSensor::kRedTreshold;
              break;
          case 1:
              shouldMoveBackward = rgbData.green > ColorSensor::kGreenTreshold;
              break;
          case 2:
              shouldMoveBackward = rgbData.blue > ColorSensor::kBlueTreshold;
              break;
          case 3:
              shouldMoveBackward = rgbData.red > ColorSensor::kYellowTreshold_R && rgbData.blue > ColorSensor::kYellowTreshold_G;
              break;
    }
    

    lineSensor->readDataFromSide(Front);
    sideDetected_[2] = lineSensor->lineDetectedFromSide();
    // lineSensor->readDataFromSide(Back);
    //sideDetected_[3] = lineSensor->lineDetectedFromSide();

    // Check if the robot is on a black square
    bool isOnBlackSquare = sideDetected_[2] == Front;
    //If shouldMoveBackward is true and the robot is on a black square, move backward
    if (shouldMoveBackward && !isOnBlackSquare) {
        moveDirection(BACKWARD, robotAngle_);
        outOfColor = true;
    } 
    // If outOfColor is true and shouldMoveBackward becomes false, set start_search to true
    else if(outOfColor && isOnBlackSquare){  
        start_search = true;
        outOfColor = false; // Reset outOfColor
    }
    // Otherwise, move in the specified direction
    else {
        moveDirection(direction, robotAngle_);
    } 
}





void Movement::driveToTarget(float coord_x){
//The idea is to move to leftmost or rghtmost based on globalPosX varable from 0 to 7 representng the map range
//Then while moving chec if corrd_x s receved from a color detecton model. 
//Once detected the color. Move based on the error proportonal if the coord_x is right in the middle (0)

    
    int xError = 0 - coord_x;

    Direction direction;
    if (!(xError < kCentered2Image)) {
        direction = (coord_x < 0) ? TOLEFT : TORIGHT; 
    } else {
        stop();
    }

    float speedFactor = 0.1; // Adjust this value as needed
    float speed = abs(xError) * speedFactor;

    moveDirection(direction, robotAngle_, speed, false);

}

void Movement::moveDirection(Direction direction, const double angleOffset, double speed, bool flag){
    switch (direction){
        case FORWARD: //OK
          lineSensor->readDataFromSide(Right);
          sideDetected_[0] = lineSensor->lineDetectedFromSide();
          lineSensor->readDataFromSide(Left);
          sideDetected_[1] = lineSensor->lineDetectedFromSide();
          // Change linear velocity sign (+/-) if Angle offset is given
          linear_x_ = (robotAngle_ == angleOffset) ? speed : speed;
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
          linear_x_ = (robotAngle_ == (angleOffset + 180)) ? -speed : -speed;
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
          linear_y_ = (robotAngle_ == angleOffset) ? speed : speed;
          // Set linear_x_ based on side detection
          if(sideDetected_[2] == Front && sideDetected_[3] == None){
              linear_x_ = movementKp * kMaxLinearX;
              Serial.println("BACKWARD");

          }
          else if(sideDetected_[3] == Back && sideDetected_[2] == None){
              linear_x_ = -movementKp * kMaxLinearX;
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
          linear_y_ = (robotAngle_ == (angleOffset + 180)) ? -speed : -speed;
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


void Movement::GoToSquare(){

}

bool Movement::detectedTilefromRaspi(){
  return this->detect_tile //change to false after testing
  }

bool Movement::detectedCubefromRaspi(){
//  if(this->detected_cube){
//     return true;
//   }
  return false;
}

void Movement::updatePIDKinematics(double fl_speed, double fr_speed, double bl_speed, double br_speed) {
  front_left_motor_.stableRPM(fl_speed);
  front_right_motor_.stableRPM(fr_speed);
  back_left_motor_.stableRPM(bl_speed);
  back_right_motor_.stableRPM(br_speed);
}

int Movement::getCurrentPosX(){
  return globalPosX_;
}


Direction Movement::getDirectionState(){
  return globalDirection_;
}

int Movement::getSquareCounter(){
  return squaresCount;
}

void Movement::setGlobalPosX(int globalPosX){
  globalPosX_ = globalPosX; 
}

void Movement::setSquareCounter(int squares){
  squaresCount = squares;
}

int Movement::getCurrentPosY(){
  return globalPosY_;
}

void Movement::setGlobalPosY(int globalPosY){
  globalPosY_ = globalPosY;
}

float Movement::getRobotAngle(){
  // bno->updateBNO();
  // float angle = bno->getYaw();
  Serial.println(robotAngle_);
  return robotAngle_;
}
