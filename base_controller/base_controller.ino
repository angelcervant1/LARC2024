#include "Movement.h"

Movement *robot = nullptr;
BNO *bnoInstance = nullptr;
LineSensor *myLineSensor = nullptr;
bool CHECK_PID = true;
bool CHECK_MOTORS = false;
bool CHECK_LINES = false;

/////////////////////////////////////remove after testing///////////////////////////////////////////////////  
unsigned long curr_millis = 0;
unsigned long prev_millis = 0;
int iteration = 0;
double angleOffset = 160.0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveForward(Movement *robot) {
    robot->orientedMovement(0.0, 0.35, 0.0);
    Serial.println("Moving Forward");
}

void moveLeft(Movement *robot) {
    robot->orientedMovement(0.35, 0.0, 0.0);
    Serial.println("Moving Left");
}
 
void moveRight(Movement *robot) { 
    robot->orientedMovement(0.0, -0.35, 0.0);
    Serial.println("Moving Right");
}

void moveBackward(Movement *robot) {
    robot->orientedMovement(-0.35, 0.0, 0.0);
    Serial.println("Moving Backwards");
}

void setup() {
    Serial.begin(57600);
    bnoInstance = new BNO(); 
    myLineSensor = new LineSensor();
    robot = new Movement(bnoInstance, myLineSensor); 
    robot->initEncoders();
}

/*
STATE MACHINE

1.- Rotate until localize color pattern
2.- Calculate X and Z coords from the color which drive towards to.
3.- Drive towards the color without minding counting squares
4.- When in front of the color, then rotate 180 degrees
5.- Start to search for color cubes 
6.- Drive with kinematics velocities towards the cube
7.- Grab the cube
8.- Turn 180 degrees and then let go the cube

*/

Direction movementVector[4] = {FORWARD, TOLEFT, BACKWARD, TORIGHT};

void loop() {
    if (CHECK_PID || CHECK_LINES || CHECK_MOTORS) {
    }
    if (CHECK_PID) {
        robot->setRobotAngle(angleOffset);
        curr_millis = millis();
        if (curr_millis - prev_millis >= 3500) {
            prev_millis = curr_millis;
            iteration++; 
            //angleOffset += 180;
            if(angleOffset >= 360)
                angleOffset = 0;
            if (iteration > 3){
                iteration = 0; 
            }
        }
        //void (*movementFunctions[])(Movement *) = {moveForward, moveLeft, moveRight, moveBackward};
        //movementFunctions[iteration](robot);
        robot->moveDirection(movementVector[iteration], angleOffset);
         //robot->orientedMovement(0.0 , 0.4, 0.0);
    }
    if (CHECK_LINES) {
        //myLineSensor.readDataFromSide(Front);
        //myLineSensor.readDataFromSide(Left);
        myLineSensor->readAllData();
    }

    if (CHECK_MOTORS) {
        while (1) {
        }
    }
    }
    
