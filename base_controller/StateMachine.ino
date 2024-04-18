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


//////////////////////////////////Main Function//////////////////////////////////////


void Encoder::handleEncoder(Motor &motor, int sign = 1) {
  int op_sign = sign == 1 ? -1 : 1;
  motor.setEncodersDir((int)(digitalRead(motor.getEncoderOne()) == HIGH ? sign : op_sign));
  motor.setPidTicks(motor.getPidTicks() + 1);
  motor.setOdomTicks(motor.getOdomTicks() + (motor.getEncodersDir()));
}

//////////////////////////////////Motor Functions//////////////////////////////////////
void Encoder::backLeftEncoder() {
  handleEncoder(robot->back_left_motor_, 1);
}

void Encoder::backRightEncoder() {
  handleEncoder(robot->back_right_motor_, -1);
}

void Encoder::frontLeftEncoder() {
  handleEncoder(robot->front_left_motor_, 1);
}

void Encoder::frontRightEncoder() {
  handleEncoder(robot->front_right_motor_, -1);


}

/// STATES ///  

void findOrigin(){
    robot->setRobotAngle(angleAmount); // read from rasp. Angle gonna be increasing until found a color paper 
    if(!robot->angleOffsetReached){
        robot->orientedMovement(0.0, 0.0, 0.0);
    }
    else if (robot->detectedTilefromRaspi()){
        robot->hardStop();
        robot->setRobotAngle(angleAmount);
        robot->orientedMovement(0.0, 0.0, 0.0);
        // robot->setGlobalPosX(start_pos_x); //this is the data received by serial with the x_coord
        currentState = FIND_EMPTY_PATH;
        robot->setInitialRobotAngle(angleAmount);
    }          
    else{
        angleAmount += 90;
    } 
}

void findEmptyPath(){
      if(robot->getCurrentPosX() > 3 && !(robot->getCurrentPosX() == 6)){
            robot->moveDirection(TORIGHT, (6 - robot->getCurrentPosX()), angleAmount);
        } else if(robot->getCurrentPosX() < 3 && !(robot->getCurrentPosX() == 0)){
                robot->moveDirection(TOLEFT, robot->getCurrentPosX(), angleAmount);
        } else {
            robot->hardStop();
            currentState = DRIVE_TO_COLOR;
        }
}

void driveToColor(){
    if(robot->start_search){
        robot->hardStop();
        currentState = ROTATE_180;
        prev_pos_x = robot->getCurrentPosX(); 
        robot->start_search = false;
        robot->angleOffsetReached = false;
    }
    else{
        robot->start_search = false;
        switch(robot->getCurrentPosX()){
            case 0:
                robot->driveToColor(0, FORWARD, GREEN);
                // Serial.println("GREEN");
            break; 

            case 3:
                robot->driveToColor(3, FORWARD, RED);
                // Serial.println("RED");
            break;
            
            case 6:
                robot->driveToColor(6, FORWARD, GREEN);
                // Serial.println("GREEN");

            break;
        }
    }
}

void rotate_180(){
    robot->setRobotAngle(fmod(angleAmount + 180, 360));
    // Serial.print("Angle  Offset: ");
    // Serial.println(robot->getRobotAngle());
    if(robot->angleOffsetReached){
        currentState = SEARCH_CUBE;
        //robot->setSquareCounter(0);
        robot->setGlobalPosX(prev_pos_x);
    }
    else{
        robot->orientedMovement(0.0, 0.0, 0.0);               
    }
    //rotate from previous position

}

void searchCube(){
    if (robot->detectedCubefromRaspi()) {  
        robot->hardStop(); 
        currentState = DRIVE_TO_CUBE;
        prev_pos_x = robot->getCurrentPosX(); 
        robot->detected_cube = false;
    } 

    else if (!fullScanCompleted) {
        // Move one square at a time based on the current position
        switch (robot->getCurrentPosX()) {
            case 0:
                if (!startingScanFrom6) {
                    startingScanFrom0 = true;
                    startingScanFrom6 = false;
                    // Move to the left to start scanning
                    if(!fromOtherSide)
                        robot->moveDirection(TOLEFT, 1, robot->getRobotAngle());
                    else
                        robot->moveDirection(TORIGHT, 1, robot->getRobotAngle());
                } else {
                    // If startingScanFrom6 is true, stop the robot and mark fullScanCompleted as true
                    robot->hardStop();
                    // fullScanCompleted = true;
                }
                break;
            case 1:
            case 2:
            case 4:
            case 5:
                if (startingScanFrom0)
                    robot->moveDirection(TOLEFT, 1, robot->getRobotAngle());
                else if (startingScanFrom6)
                    robot->moveDirection(TORIGHT, 1, robot->getRobotAngle());
                break;
            case 3:
                // If not startingScanFromRight, continue scanning from the left
                if (!(startingScanFrom0 || startingScanFrom6)) {
                    while (robot->getCurrentPosX() != 0)
                        robot->moveDirection(TORIGHT, 1, robot->getRobotAngle());

                } else if (startingScanFrom0) {
                    robot->moveDirection(TOLEFT, 1, robot->getRobotAngle());
                } else if (startingScanFrom6) {
                    robot->moveDirection(TORIGHT, 1, robot->getRobotAngle());
                }
                break;
            case 6:
               if (!startingScanFrom0) {
                    startingScanFrom0 = false;
                    startingScanFrom6 = true;
                    // Move to the left to start scanning
                    if(!fromOtherSide)
                        robot->moveDirection(TORIGHT, 1, robot->getRobotAngle());
                    else
                        robot->moveDirection(TOLEFT, 1, robot->getRobotAngle());                } else {
                    // If startingScanFrom6 is true, stop the robot and mark fullScanCompleted as true
                    robot->hardStop();
                    // fullScanCompleted = true;
                }
                break;
        }

} else {
        robot->setGlobalPosY(0);
        //If no cube detected from backwards, then move forward to start search on the other side
        while(robot->getCurrentPosY() != 4)
            robot->moveDirection(FORWARD, 1, robot->getRobotAngle());
            robot->setRobotAngle(robot->getRobotAngle() + 180);
        if (!robot->angleOffsetReached)
            robot->orientedMovement(0.0, 0.0, 0.0);
        else {
            fullScanCompleted = true;
            fromOtherSide = true;
        }
    }
}

void driveToCube(){
 
    if(robot->inFrontOfCube){
        currentState = GRAB_CUBE;
    }
    else{
        float targetXCoord = robot->getCubeCoordFromRaspi(); //send midpoint from image
        robot->driveToTarget(targetXCoord);
    }

}

void grabCube(){
    myGripper->sequenceDown(curr_millis);
    //set closese square to False after grabbng the cube
}


void enterClosestSquare(){
    if(prev_pos_x < 3 && !robot->closestSquare){
        robot->GoToSquare(TOLEFT, robot->getRobotAngle());
    } else if(prev_pos_x > 3 && !robot->closestSquare){
        robot->GoToSquare(TORIGHT, robot->getRobotAngle());
    } else{
        robot->hardStop();
        currentState = ROTATE_SEARCH_COLOR;
    }
    
}

void goToPosition(){
//how to go to a color position when grabbed the cube? ?
//we also need the respective cube color to know where 
//the robot should do

//check if raspy can handle the set pos X on each case

}

void releaseCube(){
    myGripper->sequenceUp(curr_millis);
}

void tests(){
  if (CHECK_PID) {
            robot->getRobotAngle();
            if(robot->getSquareCounter() == squares){
                iteration++;
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
            }   
        }

        if (CHECK_LINES) {
            //myLineSensor->readAllData();
            //robot->getRobotAngle();
            //colorSensor->getRgbData();
        }

        if (CHECK_ODOMETRY) {

        }

        if (CHECK_GRASP) {
            //myGripper->upLevel(0);
            myGripper->sequenceUp(curr_millis);
        }


    } 
