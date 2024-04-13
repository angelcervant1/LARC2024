#include "Movement.h"

Movement *robot = nullptr;
BNO *bnoInstance = nullptr;
LineSensor *myLineSensor = nullptr;
ColorSensor *myColorSensor = nullptr;
Gripper myGripper;

bool CHECK_PID = true;
bool CHECK_ODOMETRY = false;
bool CHECK_LINES = false;
bool CHECK_GRASP = false;

/////////////////////////////////////remove after testing///////////////////////////////////////////////////  
unsigned long curr_millis = 0;
unsigned long prev_millis = 0;
int iteration = 0;
double angleOffset = 0.0;
double squares = 1;
uint8_t start_pos_x = 0;
Direction movementVector[5] = {FORWARD, TOLEFT, BACKWARD, TORIGHT, STOP};
String currentState = "TESTS", incomingState = "" ;
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
    Wire.begin();
    Serial.begin(57600);
    //Serial.setTimeout(0.1);
    bnoInstance = new BNO(); 
    myLineSensor = new LineSensor();
    myColorSensor = new ColorSensor();
    myGripper = Gripper(); 
    robot = new Movement(bnoInstance, myLineSensor, myColorSensor); 
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

void loop() {
    
    if (Serial.available() > 0) {
       incomingState = Serial.readString();
       //Serial.println(incomingState);
        if (incomingState.equals("FIND_ORIGIN")) {
            currentState = "FIND_ORIGIN";
        } else if (incomingState.equals("FIND_EMPTY_PATH")) {
            currentState = "FIND_EMPTY_PATH";
        } else if (incomingState.equals("DRIVE_TO_COLOR")) {
            currentState = "DRIVE_TO_COLOR";
        } else if (incomingState.equals("ROTATE_180")) {
            currentState = "ROTATE_180";
        } else if (incomingState.equals("SEARCH_CUBE")) { //REMEMBER TO CHEC FROM SIDE TO SIDE FOR EASER APPROACH
            currentState = "SEARCH_CUBE";
        } else if (incomingState.equals("DRIVE_TO_CUBE")) {
            currentState = "DRIVE_TO_CUBE";
        } else if (incomingState.equals("GRAB_CUBE")) {
            currentState = "GRAB_CUBE";
        } else if (incomingState.equals("ENTER_CLOSEST_SQUARE")) {
            currentState = "ENTER_CLOSEST_SQUARE";
        } else if (incomingState.equals("ROTATE_SEARCH_COLOR")) {
            currentState = "ROTATE_SEARCH_COLOR";
        } else if (incomingState.equals("RELEASE_CUBE")) {
            currentState = "RELEASE_CUBE";
        } else {
        }
    }

    if (currentState.equals("TESTS")) {
        if (CHECK_PID) {
            robot->setRobotAngle(angleOffset);
            if(robot->getSquareCounter() == squares){
                iteration++;
                start_pos_x = robot->getCurrentPosX();
                robot->setSquareCounter(0);
                //reset movement once reached squares goal
            }
            if(iteration > 4){
                robot->stop();
            }
            else{
                robot->moveDirection(movementVector[iteration], squares, angleOffset, start_pos_x);
                //robot->moveDirection(movementVector[iteration], angleOffset);
            }
        }
        if (CHECK_LINES) {
            //myLineSensor->readAllData();
            //robot->getRobotAngle();
            myColorSensor->getRGBData();
        }
        if (CHECK_ODOMETRY) {
 
        }
        if (CHECK_GRASP){

        }
    } else if (currentState.equals("FIND_ORIGIN")) {
                angleOffset += 5; // read from rasp. Angle gonna be increasing until found a color paper 
                robot->setRobotAngle(angleOffset);
                robot->orientedMovement(0.0, 0.0, 0.0);
    } else if (currentState.equals("FIND_EMPTY_PATH")) {
        //once detected x_tile move towards center. leftmost or rightmost
        //need to get track of squares moved
    } else if (currentState.equals("DRIVE_TO_COLOR")) {
        //send a direction to move and stop until detected the color
    } else if (currentState.equals("ROTATE_180")) {
                angleOffset += 5; //read from rasp
                robot->setRobotAngle(angleOffset);
                robot->orientedMovement(0.0, 0.0, 0.0);
                //rotate from previous position
    } else if (currentState.equals("SEARCH_CUBE")) {
        //when already at a key point (center leftmost or rightmost) and looking towards shelfs search for cubes x_coords
    } else if (currentState.equals("DRIVE_TO_CUBE")) {
        //move onto a direction proportional to x_coord error. until cube right infront
        //need to get odometry for the movement to reverse it when grabing the cube
    } else if (currentState.equals("GRAB_CUBE")) {

    } else if (currentState.equals("ENTER_CLOSEST_SQUARE")) {

    } else if (currentState.equals("ROTATE_SEARCH_COLOR")) {

    } else if (currentState.equals("RELEASE_CUBE")) {

    } else {

    }
}


    
