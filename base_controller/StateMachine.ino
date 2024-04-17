
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
        robot->stop();
        robot->setRobotAngle(angleAmount);
        robot->orientedMovement(0.0, 0.0, 0.0);
        robot->setGlobalPosX(start_pos_x); //this is the data received by serial with the x_coord
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
            robot->stop();
            currentState = DRIVE_TO_COLOR;
        }
}

void driveToColor(){
    if(robot->start_search){
        robot->stop();
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
                Serial.println("GREEN");
            break; 

            case 3:
                robot->driveToColor(3, FORWARD, RED);
                Serial.println("RED");
            break;
            
            case 6:
                robot->driveToColor(6, FORWARD, GREEN);
                Serial.println("GREEN");

            break;
        }
    }
}

void rotate_180(){
    robot->setRobotAngle(fmod(angleAmount + 180, 360));
    Serial.print("Angle  Offset: ");
    Serial.println(robot->getRobotAngle());
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
 if (robot->detectedCubefromRaspi()) {  robot->stop(); } 
    
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
                    robot->stop();
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
                    robot->stop();
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
    uint8_t targetXCoord = robot->detectedCubefromRaspi(); //send midpoint from image
    robot->driveToTarget(targetXCoord);

}

void grabCube(){
    myGripper->sequenceUp(curr_millis);
}


void enterClosestSquare(){

}

void rotateSearchColor(){

}

void releaseCube(){

}

void tests(){
  if (CHECK_PID) {
            //robot->getRobotAngle();
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
            //colorSensor->getRGBData(rgbData);
        }

        if (CHECK_ODOMETRY) {

        }

        if (CHECK_GRASP) {
            //myGripper->upLevel(0);
            myGripper->sequenceUp(curr_millis);
        }


    } 
