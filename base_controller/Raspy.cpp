#include "Raspy.h"

Raspy::Raspy(BNO *bno, LineSensor *line, ColorSensor *color, Movement *robot, Gripper *gripper){
    _bno = bno;
    _line = line;
    _color = color;
    _robot = robot;
    _gripper = gripper;
    flag = "";
}

void Raspy::readSerial() {
    static uint8_t buffer[18];
    static uint8_t index = 0;
    static uint8_t packet_size = 0;
    static uint8_t command = 0;
    static uint8_t check_sum = 0;
    
    while (Serial.available()) {
        buffer[index++] = Serial.read();

        // Check packet header
        if (index == 1 && buffer[0] != 0xFF) {
            index = 0;
            packet_size = 0;
            command = 0;
        }
        if (index == 2 && buffer[1] != 0xAA) {
            packet_size = 0;
            command = 0;
            index = 0;
        }
        
        // Read packet size and command
        if (index == 4) {
            packet_size = buffer[2];
            command = buffer[3];
        }
        
        // Check if the entire packet has been received
        if (index == 3 + (packet_size) + 1) {
            check_sum = buffer[index - 1];
            if (check_sum != command + 1) {
                // Checksum error
                index = 0;
                packet_size = 0;
                command = 0;
                continue;
            }
            // Execute the command
            executeCommand(packet_size, command, &buffer[4]);
            
            // Reset index and packet_size
            index = 0;
            packet_size = 0;
        }
    }
    // if serial is not available, start a counter to stop the robot if nothing is received in a time frame
    // unactive_time_ = millis();

}

void Raspy::executeCommand(uint8_t packet_size, uint8_t command, uint8_t* buffer) {
    switch (command) {
        case 0x13: // Hardware Version 
            if (packet_size == 1) { // Check packet size
                uint32_t version[] = {1};
                writeSerial(true, (uint8_t*)version, sizeof(version));
            }
        break;
        case 0x00: // Baud
            if (packet_size == 1) { // Check packet size
                uint32_t baud[] = {115200};
                writeSerial(true, (uint8_t*)baud, sizeof(baud));
            }
            break;
        case 0x01: //Color Detected
            int angleAmount = 0; 
            if (packet_size == 1) { // Check packet size
                  
                writeSerial(true, nullptr, 0);
            }
            break;
        case 0x02: // rotate
            if (packet_size == 5) { // Check packet size
                double angleAmount;
                memcpy(&angleAmount, buffer, sizeof(angleAmount));
                _robot->setRobotAngle(angleAmount); // read from rasp. Angle gonna be increasing until found a color paper 
                if(!_robot->angleOffsetReached){
                    _robot->orientedMovement(0.0, 0.0, 0.0);
                }
                writeSerial(true, nullptr, 0);
            }
            break;
        case 0x03: // Location 
            if (packet_size == 3){
                uint8_t start_x_pos;
                uint8_t color;
                memcpy(&start_x_pos, buffer, sizeof(start_x_pos));
                memcpy(&color, buffer + sizeof(start_x_pos), sizeof(color));
                _robot->setGlobalPosX(start_x_pos);
                _robot->initalTileColor = color;
                flag = "TESTS"
            }
            writeSerial(true, nullptr, 0);
            break;
    }
}

void Raspy::writeSerial(bool success, uint8_t* payload, int elements) {
  uint8_t ack = success ? 0x00 : 0x01;
  Serial.write(0xFF);
  Serial.write(0xAA);
  Serial.write(sizeof(uint8_t) * elements + 1); // Packet size
  Serial.write(ack); // ACK
  
  // Send payload bytes
  for (size_t i = 0; i < elements; i++) {
    Serial.write(payload[i]);
  }

  //Serial.write(0x00); // Footer
  Serial.flush();

}

String Raspy::raspystate(){
    Raspy::readSerial();
    return flag;
}
