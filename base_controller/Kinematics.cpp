/*
  Copyright (c) 2016, Juan Jimeno
  Source: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
   Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
   Neither the name of  nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific
  prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORTPPIPI (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "Kinematics.h"

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, float kWheelBase, float kWheelTrack)
{
  //this->motor = motor;
  circumference_ = PI * wheel_diameter;
  max_rpm_ = motor_max_rpm;
  kWheelBase_ = kWheelBase;
  kWheelTrack_ = kWheelTrack;
}

Kinematics::output Kinematics::getRPM(float linearX, float linearY, float angularZ)
{
  
  // //Distance from the center of the robot to the center of the wheels
  // float R = 0.33;
  // //bno->updateBNO();
  // // //Vt = ω * radius
  // tangential_vel_ = angular_vel_z_mins_ * lr_wheels_dist_;
  // x_rpm_ = linear_vel_x_mins_ / circumference_;
  // y_rpm_ = linear_vel_y_mins_ / circumference_;
  // tan_rpm_ = tangential_vel_ / circumference_;

  Kinematics::output rpm;
  
    float wheelPosX = kWheelBase_/2;
    float wheelPosY = kWheelTrack_/2;

    float frontLeftSpeed = linearX - linearY - angularZ*(wheelPosX + wheelPosY);
    float frontRightSpeed = linearX - linearY + angularZ*(wheelPosX + wheelPosY);
    float backLeftSpeed = linearX + linearY - angularZ*(wheelPosX + wheelPosY);
    float backRightSpeed = linearX + linearY + angularZ*(wheelPosX + wheelPosY);

    rpm.motor1 = frontLeftSpeed * 60 / circumference_;
    rpm.motor2 = frontRightSpeed * 60 / circumference_;
    rpm.motor3 = backLeftSpeed * 60 / circumference_;
    rpm.motor4 = backRightSpeed * 60 / circumference_;

  return rpm;
}
