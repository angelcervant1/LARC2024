#include "Movement.h"
#include "BNO.h"
#include "Plot.h"

Movement *robot = nullptr;
BNO *bnoInstance = nullptr;
LineSensor myLineSensor;
bool ROS_ENABLE = false;
bool CHECK_PID = true;
bool CHECK_MOTORS = false;
bool CHECK_LINES = false;

/////////////////////////////////////remove after testing///////////////////////////////////////////////////
unsigned long curr_millis = 0;
unsigned long prev_millis = 0;
int iteration = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void moveForward(Movement *robot) {
    robot->orientedMovement(0.0, 0.5, 0.0);
    Serial.println("Moving Left");
}

void moveLeft(Movement *robot) {
    robot->orientedMovement(0.5, 0.0, 0.0);
    Serial.println("Moving Forward");
}

void moveRight(Movement *robot) {
    robot->orientedMovement(0.0, -0.5, 0.0);
    Serial.println("Moving Right");
}

void moveBackward(Movement *robot) {
    robot->orientedMovement(-0.5, 0.0, 0.0);
    Serial.println("Moving Backwards");
}

void setup() {
    Serial.begin(57600);
    bnoInstance = new BNO(); 
    robot = new Movement(bnoInstance); 
    robot->initEncoders();
}

void loop() {
    if (CHECK_PID || CHECK_LINES || CHECK_MOTORS) {
    }
    
    if (CHECK_PID) {
        robot->setRobotAngle(0);
        curr_millis = millis();
        if (curr_millis - prev_millis >= 3500) {
            prev_millis = curr_millis;
            iteration++;
            if (iteration > 3) {
                iteration = 0; 
            }
        }
        
        void (*movementFunctions[])(Movement *) = {moveForward, moveLeft, moveRight, moveBackward};

        movementFunctions[iteration](robot);
    }

    if (CHECK_LINES) {
        //myLineSensor.readDataFromSide(Front);
        //myLineSensor.readDataFromSide(Left);
        myLineSensor.readAllData();
    }

    if (CHECK_MOTORS) {
        while (1) {
        }
    }
}
