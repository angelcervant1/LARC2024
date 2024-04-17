#include "Gripper.h"

Gripper::Gripper() {
    // Initialize servo motor
    myServoR.attach(servoRAnalogPin); 
    myServoL.attach(servoLAnalogPin); 
    releaseCube();
    myStepper = AccelStepper(1, Nema_Steep, Nema_Direction);
    pinMode(limitSwitchPin, INPUT);
    //digitalWrite(limitSwitchPin, 0);
    myStepper.setMaxSpeed(10000);
    myStepper.setAcceleration(5000);
    Nivel=0;
    // last_time = prev_millis;
    
    // Assuming the first pin in the array is for the servo
// Assuming the first pin in the array is for the servo
}

void Gripper::downLevel(const uint8_t level) {
 
}

void Gripper::upLevel(const uint8_t level) {
    // Calculate the target position based on the level
    int targetPosition = Paso * level - Ajuste;
    // Move the stepper motor to the target position
    //myStepper.move(100);
    myStepper.runToPosition();
    while (level < 19) {
          myStepper.moveTo(targetPosition);
          myStepper.runToPosition();
          Serial.print("Move to 0 Pos");
          myStepper.move(0);
      }
}


void Gripper::downSteps(int steps) {
}

void Gripper::upSteps(int steps) {

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
    
    while(digitalRead(limitSwitchPin)==LOW && !isHome){
        myStepper.move(-2000);
        myStepper.runToPosition();
        Ajuste=Ajuste+2000;
        Serial.print("Going Home...");
    }
    isHome = true;
    stop();
}

void Gripper::stop() {
        // Stop the stepper motor
    pinMode(Nema_Direction,OUTPUT);
    digitalWrite(Nema_Direction,LOW);
    myStepper.setMaxSpeed(0);
  }

void Gripper::sequenceUp(unsigned long curr_milis){
  current_time = curr_milis;

  if (!gripping && !releasing) {
        upLevel(0);
        grabCube();
        gripping = true; 
        graspStartTime = current_time;  
        Serial.println("GRAB");
    } else if (gripping && !releasing && current_time - graspStartTime >= 3000) {
        releaseCube();
        releasing = true;  
        releaseStartTime = current_time;  
        Serial.println("RELEASE");
    } else if (releasing && current_time - releaseStartTime >= 3000) {
        gripping = false;
        releasing = false;
        upLevel(6);
        Serial.println("WAIT OVER");
    }
}


void Gripper::sequenceDown(unsigned long curr_millis){
  current_time = curr_millis;

}
