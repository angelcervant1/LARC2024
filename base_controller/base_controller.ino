#include "Movement.h"
#include "BNO.h"
#include "Plot.h"

Movement *robot = nullptr;
BNO *bnoInstance = nullptr; 
bool ROS_ENABLE = false;
bool CHECK_PID = true;
bool CHECK_MOTORS = false;
bool CHECK_ENCODERS = false;
double data = 0.0;
double setpoint = 100.0;

/////////////////////////////////////remove after testing///////////////////////////////////////////////////

float velocity_vector = 0;
float target_angle = 0;
float curr_angle = 0;
float max_rpm = 40;
float max_lin_vel = 0.20; ///Maximum linear velocity of the robot is .2 m/s approximately
unsigned long curr_millis = 0;
unsigned long prev_millis = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(57600);
    //Serial2.begin(575600);
    bnoInstance = new BNO(); 
    robot = new Movement(bnoInstance); 
    robot->initEncoders();
}


void loop() {
    if (CHECK_PID || CHECK_ENCODERS || CHECK_MOTORS) {
        Serial.begin(57600);
    }
    if (CHECK_PID) {
        curr_millis = millis();
        prev_millis = curr_millis;
        while (1) {
            curr_millis = millis();
            if((curr_millis - prev_millis) < 3000){
              robot->orientedMovement(0.0, 0.4, 0.0);
              robot->setRobotAngle(0);
              Serial.println("Moving Forward");  
            }
            else if((curr_millis - prev_millis) < 6000){
              robot->orientedMovement(0.4, 0.0, 0.0);
              Serial.println("Turning Right");
            }
            else if((curr_millis - prev_millis) < 9000){
              robot->orientedMovement(0.0, -0.4, 0.0);
              Serial.println("Moving Backward");
            }
            else if((curr_millis - prev_millis) < 12000){
              robot->orientedMovement(-0.4, 0.0, 0.0);
              Serial.println("Turning Left");
            }
            if ((curr_millis - prev_millis) >= 12000) {
              prev_millis = curr_millis;
            }
        }
    }

    if (CHECK_ENCODERS) {
        while (1) {
        }
    }

    if (CHECK_MOTORS) {
        while (1) {
        }
    }
}
