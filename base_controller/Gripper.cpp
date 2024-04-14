#include "Gripper.h"

Gripper::Gripper() : myStepper(stepsPerRevolution, 49, 48) {
    // Initialize servo motor
    myServoR.attach(servoRAnalogPin); 
    myServoL.attach(servoLAnalogPin); 
    releaseCube();
    
    // Assuming the first pin in the array is for the servo
// Assuming the first pin in the array is for the servo
}

void Gripper::downLevel(const uint8_t level) {
 
    myStepper.step(-level * stepsPerLevel); // Assuming stepsPerLevel is defined elsewhere
}

void Gripper::upLevel(const uint8_t level) {

    myStepper.step(level * stepsPerLevel); // Assuming stepsPerLevel is defined elsewhere
}

void Gripper::downSteps(const uint8_t steps) {
    myStepper.step(-steps);
}

void Gripper::upSteps(const uint8_t steps) {
    myStepper.step(steps);
}

void Gripper::releaseCube() {
  
  myServoL.write(degreesToRelease); // Set servo L to desired position// Assuming 90 degrees is the closed position, adjust as needed
  myServoR.write(degreesToGrab); // Set servo L to desired position// Assuming 90 degrees is the closed position, adjust as needed

}

void Gripper::grabCube(){

  myServoR.write(degreesToRelease); // Set servo R to desired position
  myServoL.write(degreesToGrab); // Set servo L to desired position


}

void Gripper::stop() {
    // Stop the stepper motor
    myStepper.step(0);
}
