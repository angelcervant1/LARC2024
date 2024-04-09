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
int iteration = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveForward(Movement *robot) {
    robot->orientedMovement(0.0, 0.3, 0.0);
    Serial.println("Moving Forward");
}

void moveLeft(Movement *robot) {
    robot->orientedMovement(0.3, 0.0, 0.0);
    Serial.println("Moving Left");
}

void moveRight(Movement *robot) {
    robot->orientedMovement(0.0, -0.3, 0.0);
    Serial.println("Moving Right");
}

void moveBackward(Movement *robot) {
    robot->orientedMovement(-0.3, 0.0, 0.0);
    Serial.println("Moving Backwards");
}

void setup() {
    Serial.begin(57600);
    //Serial2.begin(575600);
    bnoInstance = new BNO(); 
    robot = new Movement(bnoInstance); 
    robot->initEncoders();
}

void loop() {
    if (CHECK_PID || CHECK_ENCODERS || CHECK_MOTORS) {
    }
    
    if (CHECK_PID) {
        curr_millis = millis();
        if (curr_millis - prev_millis >= 4000) {
            prev_millis = curr_millis;
            iteration++;
            if (iteration > 3) {
                iteration = 0; 
            }
        }
        
        // Array of function pointers
        void (*movementFunctions[])(Movement *) = {moveForward, moveLeft, moveRight, moveBackward};

        // Call the function based on iteration
        movementFunctions[iteration](robot);
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
