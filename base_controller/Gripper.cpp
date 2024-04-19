#include "Gripper.h"

Gripper::Gripper() {
    // Initialize servo motor
    myServoR.attach(servoRAnalogPin); 
    myServoL.attach(servoLAnalogPin); 
    myServoR.write(0);
    myServoL.write(0);
    releaseCube();
    myStepper = AccelStepper(1, Nema_Steep, Nema_Direction);
    pinMode(limitSwitchPin, INPUT);
    //digitalWrite(limitSwitchPin, 0);
    myStepper.setMaxSpeed(10000);
    myStepper.setAcceleration(8000);
    // last_time = prev_millis;
    
    // Assuming the first pin in the array is for the servo
    // Assuming the first pin in the array is for the servo
}

void Gripper::downLevel(uint8_t level) {
  
}

void Gripper::upLevel(uint8_t level) {
    digitalWrite(Nema_Direction, HIGH);
    int targetPosition = (Paso * level - Ajuste);
    if (level < 19) {
        myStepper.moveTo(targetPosition);
        myStepper.run();
    }
}





void Gripper::downSteps(int steps) {
}

void Gripper::upSteps(int time) {
    int t = time;
    while(millis() - time < 15000){
        upLevel(5);
}
}

void Gripper::releaseCube() {
  
  myServoL.write(degreesToRelease); // Set servo L to desired position// Assuming 90 degrees is the closed position, adjust as needed
  myServoR.write(degreesToGrab); // Set servo L to desired position// Assuming 90 degrees is the closed position, adjust as needed

}

void Gripper::grabCube(){

  myServoR.write(degreesToRelease); // Set servo R to desired position
  myServoL.write(degreesToGrab); // Set servo L to desired position


}
void Gripper::StepperHome(){
    digitalWrite(Nema_Direction, HIGH);
    int last_check = millis();
    int debounce = 100;
    while(true){
        bool read = digitalRead(limitSwitchPin);
        if(read){
            if (millis()-last_check>debounce){
              break;
            }
        }
        else{
            myStepper.move(-2000);
            myStepper.runToPosition();
            Ajuste=Ajuste+2000;
            last_check = millis();
        }
        // Serial.print("Going Home...");
    }
    upSteps(last_check);
    //isHome = true;
    //myStepper.setMaxSpeed(0);
    digitalWrite(Nema_Direction,LOW);
  
}

void Gripper::stop(){
    // Stop the stepper motor
    //pinMode(Nema_Direction,OUTPUT);
    digitalWrite(Nema_Direction,LOW);
    // myStepper.setMaxSpeed(0);
  }

void Gripper::sequenceUp(unsigned long curr_milis){
  current_time = curr_milis;

  if (!gripping && !releasing) {
        upLevel(0);
        releaseCube();
        gripping = true; 
        graspStartTime = current_time;  
        // Serial.println("GRAB");
    } else if (gripping && !releasing && current_time - graspStartTime >= 3000) {
        releasing = true;  
        releaseStartTime = current_time;  
        // Serial.println("RELEASE");
    } else if (releasing && current_time - releaseStartTime >= 3000) {
        gripping = false;
        releasing = false;
        upLevel(10);
        // Serial.println("WAIT OVER");
    }
}


void Gripper::sequenceDown(unsigned long curr_millis){
  current_time = curr_millis;
  if (!gripping && !releasing) {
        upLevel(0);
        grabCube();
        gripping = true; 
        graspStartTime = current_time;  
        // Serial.println("GRAB");
    } else if (gripping && !releasing && current_time - graspStartTime >= 1) {
        grabCube();
        releasing = true;  
        releaseStartTime = current_time;  
        //Serial.println("RELEASE");
    } else if (releasing && current_time - releaseStartTime >= 1) {
        gripping = false;
        //releasing = false;
        upLevel(5);
        // Serial.println("WAIT OVER");
    }
}