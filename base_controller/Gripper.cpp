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
    myStepper.setAcceleration(5500);
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
        myStepper.runToPosition();
    }
    Serial.print("COCMOCM");
    prevLevel = level;
}

void Gripper::downSteps(int steps) {
}

void Gripper::upSteps(int time) {
    // digitalWrite(Nema_Direction, HIGH);
    // int t = 0;
    // while(true){
    // Serial.print("AAAAA");
    // upLevel(5);
    // t = millis();
    // if(t - time < 6000){
    //     Serial.print("TERMNE");
    //     break;
    //     }
    // }
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
    // digitalWrite(Nema_Direction, HIGH);
    // int last_check = millis();
    // int debounce = 100;
    // while(true){
    //     bool read = digitalRead(limitSwitchPin);
    //     Serial.println(read);
    //     if(read){ 
    //         if (millis()-last_check > debounce){
    //           break;
    //         }
    //     }
    //     else{
    //         myStepper.move(-2000);
    //         myStepper.runToPosition();
    //         Ajuste=Ajuste+2000;
    //         last_check = millis();
    //     }
    //     // Serial.print("Going Home...");
    // }
    digitalWrite(Nema_Direction, HIGH);
    while(digitalRead(limitSwitchPin)==LOW){
        myStepper.move(-2000);
        myStepper.runToPosition();
        Ajuste=Ajuste+2000;
        // Serial.print("Going Home...");
    }
    // upLevel(6);
    //isHome = true;
    //myStepper.setMaxSpeed(0);
    digitalWrite(Nema_Direction,LOW);
    // digitalWrite(Nema_Direction,LOW);

    // digitalWrite(Nema_Direction, HIGH);
    // // while (true)
    // // {
    // //     Serial.print(digitalRead(limitSwitchPin));
    // // }
    // int t = digitalRead(limitSwitchPin);
    // while(!t){
    //     t = digitalRead(limitSwitchPin);
    //     Serial.print(t);
    //     myStepper.move(-2000);
    //     myStepper.runToPosition();
    //     Ajuste=Ajuste+2000;
    //     // Serial.print("Going Home...");
    // }
    //upLevel(5);
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
        //releaseCube();
        upLevel(0);
        gripping = true; 
        graspStartTime = current_time;  
        // Serial.println("GRAB");
    } else if (gripping && !releasing && current_time - graspStartTime >= 3000) {
        releaseCube();
        releasing = true;  
        releaseStartTime = current_time;  
        // Serial.println("RELEASE");
    } else if (releasing && current_time - releaseStartTime >= 3000) {
        gripping = false;
        releasing = false;
        upLevel(5);
        // Serial.println("WAIT OVER");
    }
}


void Gripper::sequenceDown(unsigned long curr_millis){
  current_time = curr_millis;
  if (!gripping && !releasing) {
        upLevel(0);
        //releaseCube();
        gripping = true; 
        graspStartTime = current_time;  
         Serial.println("GRAB");
    } else if (gripping && !releasing && current_time - graspStartTime >= 1500) {
        grabCube();
        releasing = true;  
        releaseStartTime = current_time;  
        Serial.println("GRAB");
    } else if (releasing && current_time - releaseStartTime >= 1500) {
        gripping = false;
        //releasing = false;
        upLevel(5);
        //leaveCube = true;
        Serial.println("WAIT OVER");
    }
}
