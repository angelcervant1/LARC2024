#include "Movement.h"
#include "Raspy.h"
#include "Encoder.h"

Movement *robot = nullptr;
BNO *bnoInstance = nullptr;
LineSensor *myLineSensor = nullptr;
ColorSensor *myColorSensor = nullptr;
Gripper *myGripper = nullptr;
Raspy raspy;

bool CHECK_PID = false;
bool CHECK_ODOMETRY = false;
bool CHECK_LINES = true;
bool CHECK_GRASP = false;

/////////////////////////////////////remove after testing///////////////////////////////////////////////////  
unsigned long curr_millis = 0;
unsigned long prev_millis = 0;
int iteration = 0;
double angleOffset = 0.0; //for tests
double squares = 4; 
float angleAmount = 0.0;  //for state machne
uint8_t start_pos_x = 6;
Direction movementVector[5] = {FORWARD, TOLEFT, BACKWARD, TORIGHT, STOP};
float graspStartTime = 3000;
float releaseStartTime = 3000;
bool gripping;
bool releasing;
bool reachedAngle;
int prev_pos_x = 0;
bool fullScanCompleted = false;
bool startingScanFrom0 = false;
bool startingScanFrom6 = false;
bool fromOtherSide = false;

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


enum States {
    TESTS,
    FIND_ORIGIN,
    FIND_EMPTY_PATH,
    DRIVE_TO_COLOR,
    ROTATE_180,
    SEARCH_CUBE,
    DRIVE_TO_CUBE,
    GRAB_CUBE,
    ENTER_CLOSEST_SQUARE,
    ROTATE_SEARCH_COLOR,
    RELEASE_CUBE,
    DEFAULT_STATE
};

States currentState;


void setup() {
    //Wire.begin();
    Serial.begin(115200);
    bnoInstance = new BNO(); 
    myLineSensor = new LineSensor();
    myColorSensor = new ColorSensor();
    myGripper = new Gripper(); 
    robot = new Movement(bnoInstance, myLineSensor, myColorSensor); 
    robot->initEncoders();
    robot->setGlobalPosX(start_pos_x);
    robot->angleOffsetReached = false;
    //myGripper->StepperHome();
    Serial.print("Starting");
    raspy.import(robot);
    currentState = FIND_ORIGIN; //chhange based on raspy instruction
    //currentState = TESTS;
    //currentState = DRIVE_TO_COLOR;
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


void loop() {
    raspy.readSerial();
    if(raspy.update){
        currentState = raspy.get_State();
        raspy.update = false;
    }
    curr_millis = millis();

    if (currentState == TESTS) {
        tests();
    } else if (currentState == FIND_ORIGIN) {
        findOrigin();
    } else if (currentState == FIND_EMPTY_PATH) {
        findEmptyPath();
    } else if (currentState == DRIVE_TO_COLOR) {
        driveToColor();
    } else if (currentState == ROTATE_180) {
        rotate_180();
    } else if (currentState == SEARCH_CUBE) {
        searchCube();
    } else if (currentState == DRIVE_TO_CUBE) {
        driveToCube();
    } else if (currentState == GRAB_CUBE) {
        grabCube();
    } else if (currentState == ENTER_CLOSEST_SQUARE) {
        enterClosestSquare();
    } else if (currentState == ROTATE_SEARCH_COLOR) {
        rotateSearchColor();
    } else if (currentState == RELEASE_CUBE) {
        releaseCube();
    } else {
        // Handle the default case
        // In case nothing is received from raspy
    }

    Serial.println(currentState);
    Serial.print("Global Pos X: "); Serial.println(robot->getCurrentPosX());
    Serial.print("Global Angle: "); Serial.print(robot->getRobotAngle());
}





    
