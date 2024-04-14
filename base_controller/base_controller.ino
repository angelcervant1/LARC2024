#include "Movement.h"

Movement *robot = nullptr;
BNO *bnoInstance = nullptr;
LineSensor *myLineSensor = nullptr;
ColorSensor *myColorSensor = nullptr;
Gripper *myGripper = nullptr;

bool CHECK_PID = true;
bool CHECK_ODOMETRY = false;
bool CHECK_LINES = false;
bool CHECK_GRASP = false;

/////////////////////////////////////remove after testing///////////////////////////////////////////////////  
unsigned long curr_millis = 0;
unsigned long prev_millis = 0;
int iteration = 0;
double angleOffset = 0.0;
double squares = 2;
float angleAmount = 0.0;
uint8_t start_pos_x = 3;
Direction movementVector[5] = {FORWARD, TOLEFT, BACKWARD, TORIGHT, STOP};
String currentState = "TESTS", incomingState = "" ;

static bool fullScanCompleted = false;
static bool startingScanFrom0 = false;
static bool startingScanFrom6 = false;

///////////////////////////////////// //////////////////////////////////////////////////////////////////////////////////////

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
    myColorSensor = new ColorSensor();
    myGripper = new Gripper(); 
    robot = new Movement(bnoInstance, myLineSensor, myColorSensor); 
    robot->initEncoders();
    robot->setGlobalPosX(start_pos_x);
}

/*
STATE MACHINE

1.- Rotate until localize color pattern and save current Angle as absolute offset when looking at colors
2.- Calculate X coords from color pattern detector and the color which is in front of you
3.- Estime which square to move is closer from origin. 

    Example:if robot is currently at x: 0, then just move forward until detect the color 
    accordly to that coord_x (Green in this case)
    if it is looking at red, then x: 3, so move forward. 

    Key Coords are : 0, 3, 6 (Leftmost, Rightmost, Center) which are zones where cubes cant be placed
    Basically based of the coord x, move to the closest Key Coords.

3.- Drive towards the color until TCS detects color (Based on color detected previous)
4.- Routine to move backwards to keep inside squares
5.- Rotate 180 degrees 
6.- Search for cubes and save last globalPosX before detecting a color

    Three cases
    x: 0 -> Move Left until detect a color
    x: 3 -> Go to x: 0 
    x: 6 -> Move Right until detect a color

(All into square movements)

7.- Once detected a color, try to center robot to 0 pixels coords from image. 
    Adjust linear velocity based on Error from image x_coord


8.- Move forward until bounding box is big enough to consider cube is right in front of the robot.
    Detect the color of the cube

9.- Grabbing the cube 
    Nema Motor moves down
    Servos close to grab the cube
    Nema Motor moves up

10.- Because robot has moved to be in the middle of the cube, it has to move into a square

    Check last saved globalPosX, 
    last GlobalPosX > 3 
        Then move to the Right when robot looking to shelfs or To Left if looking to colors
        until detected a black line, then compensate a little bit with +/-linear_y

    last GlobalPosX < 3
        Then move to the Left when robot looking to shelfs or To Right if looking to colors
        until detected a black line, then compensate a little bit with +/-linear_y


    After aligned in X, then we move backwards until detect a black line
    then compensate a little bit with +/- linear_x
    this way we will be inside a square

11.- Once robot into a square, then rotate to the colors once more (offsetAngle), to get the x_coord from color detector and keep track of itself 
    
        OR  

    Drive until reached a color: Red, Green, Blue, Yellow; (By ColorSensor RGB data)

    If robot sees red
     x_coord = 3;
    If robot sees Green (Mostly never will happen because cubes are majorly reached from the track center)
        Move Left 1 square;
    If robot sees Blue
        Then Move Left 1 square
        If robot sees yellow
            x_coord = 4
        If doesnt sees yellow, means it sees green
            x_coord = 0
    If robot sees yellow
        then move to Left
            If robot sees red;
                x_coord = 3;
            If robot sees blue;
                x_coord = 1
    
12.- Once known x_coord, cube must be released in the color_id x_coord.
    Cube is blue -> go to x: 5 or x: 1
    Cube is red -> go to x: 3
    Cube is green -> go to x: 0 or x: 7
    Cube is yellow -> go to x: 2 or x: 4

13.- Nema motor moves down
     Release the cube
     Nema motor moves up
     Change gripper to initial position

14.- Rotate towards the Shelfs

15.- Repeat from step 6


*/
float graspStartTime = 3000;
float releaseStartTime = 3000;
bool gripping;
bool releasing;
bool reachedAngle;

void loop() {
    // if (Serial.available() > 0) {
    //    incomingState = Serial.readString();
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
    //}         

    if (currentState.equals("TESTS")) {
        if (CHECK_PID) {
            robot->getRobotAngle();
            if(robot->getSquareCounter() == squares){
                iteration++;
                robot->setSquareCounter(0);
                start_pos_x = robot->getCurrentPosX();
                //reset movement once reached squares goal
            }
            if(iteration > 4){
                robot->stop();
            }
            else{
                if(!robot->angleOffsetReached){
                    robot->orientedMovement(0.0, 0.0, 0.0);
                }
                else{
                    robot->setRobotAngle(angleOffset);
                    robot->moveDirection(movementVector[iteration], squares, angleOffset);
                }
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
        if (CHECK_GRASP) {
            if (!gripping && !releasing) {
                // If not currently gripping or releasing, initiate the gripping process
                myGripper->grabCube();
                gripping = true;  // Set the flag indicating that we are gripping
                graspStartTime = millis();  // Record the start time of the gripping process
                Serial.println("GRAB");
            } else if (gripping && !releasing && millis() - graspStartTime >= 3000) {
                // If currently gripping and 3000 milliseconds have elapsed, release the cube
                myGripper->releaseCube();
                releasing = true;  // Set the flag indicating that we are releasing
                releaseStartTime = millis();  // Record the start time of the releasing process
                Serial.println("RELEASE");
            } else if (releasing && millis() - releaseStartTime >= 3000) {
                // If currently releasing and 3000 milliseconds have elapsed, reset gripping and releasing flags
                gripping = false;
                releasing = false;
                Serial.println("WAIT OVER");
            }
        }


    } else if (currentState.equals("FIND_ORIGIN")) {
                robot->setRobotAngle(angleAmount); // read from rasp. Angle gonna be increasing until found a color paper 
                if(!robot->angleOffsetReached){
                    robot->orientedMovement(0.0, 0.0, 0.0);
                }
                else if (robot->detectedTilefromRaspi()){
                    robot->stop();
                    robot->setRobotAngle(angleAmount);
                    robot->orientedMovement(0.0, 0.0, 0.0);
                    robot->setGlobalPosX(start_pos_x); //this is the data received by serial with the x_coord
                }          
                else{
                    angleAmount += 90;
                }  

    } else if (currentState.equals("FIND_EMPTY_PATH")) {

                if(robot->getCurrentPosX() > 3)
                    robot->moveDirection(TORIGHT, (6 - robot->getCurrentPosX()), angleAmount);
                else
                    robot->moveDirection(TOLEFT, robot->getCurrentPosX(), angleAmount);

    } else if (currentState.equals("DRIVE_TO_COLOR")) {
        //send a direction to move and stop until detected the color
                switch(robot->getCurrentPosX()){
                    case 0:
                        robot->driveToColor(0, FORWARD, GREEN);
                    break; 

                    case 3:
                        robot->driveToColor(3, FORWARD, RED);
                    break;
                    
                    case 6:
                        robot->driveToColor(6, FORWARD, GREEN);
                    break;

                }
            
    } else if (currentState.equals("ROTATE_180")) {
                robot->setRobotAngle(angleAmount + 180);
                robot->orientedMovement(0.0, 0.0, 0.0);               
                //rotate from previous position
    } else if (currentState.equals("SEARCH_CUBE")) {
    // Flag to track if the robot has completed a full scan
    
    // Check for cube detection
    if (robot->detectedCubefromRaspi()) { robot->stop(); }

    else {
    // Move one square at a time based on the current position
    switch(robot->getCurrentPosX()) {
        case 0:
            if(!startingScanFrom0){
                startingScanFrom0 = true;
                startingScanFrom6 = false;
            }
            if(startingScanFrom6){
                robot->stop();
            }
        
            break; 
        case 1:
        case 2:
        case 4:
        case 5:
            if(startingScanFrom0)
                robot->moveDirection(TOLEFT, 1, robot->getRobotAngle());
            else if(startingScanFrom6)
                robot->moveDirection(TORIGHT, 1, robot->getRobotAngle());
            // Move to the next square
            break;
        
        case 3:
            if (!fullScanCompleted) {
                // Full scan not completed, move to the LEFT to start scanning
                while (robot->getCurrentPosX() != 0) 
                    robot->moveDirection(TOLEFT, 1, robot->getRobotAngle());
            } else 
                robot->stop();

            break;

        case 6:
            if(startingScan){
                startingScan = false;
            }
            break;
        }
    }

                //The other side of this state is if not any cube detection.
                //Robot must go to the init position to search from the other side.

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


    
