#include "Gripper.h"

Gripper::Gripper() {
    // Initialize servo motor
    myServo.attach(servoAnalogPin); // Assuming the first pin in the array is for the servo
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

void Gripper::grabCube() {
    // Close the gripper to grab the cube
    myServo.write(90); // Assuming 90 degrees is the closed position, adjust as needed
}

void Gripper::stop() {
    // Stop the stepper motor
    myStepper.step(0);
}
