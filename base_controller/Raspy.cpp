
#include "Raspy.h"
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
}state;

Raspy::Raspy(){
    state = TESTS;
    tile = 7;
    color = 4;
    update = false;
}
void Raspy::import(Movement *robot){
    _robot = robot;
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
        case 0x01: // Send localization
            if (packet_size == 5) { // Check packet size
                state = FIND_ORIGIN;
                uint32_t t;
                memcpy(&t, buffer, sizeof(t));
                _robot->setGlobalPosX(t);
                uint32_t s[] = {t};
                _robot->detect_tile = true;
                this->update = true;
                writeSerial(true, (uint8_t*)s, sizeof(s));
            }
            break;
        case 0x02: // Initialize origin finding 
            if (packet_size == 1){
                this->flag = "FIND_ORIGIN";
                this->update = true;
                writeSerial(true, nullptr, 0);
            }
            break;
        case 0x04: // Cube detection
            if (packet_size == 5){
                uint32_t t; 
                memcpy(&t, buffer, sizeof(t));
                _robot->detected_cube = t;
                writeSerial(true, nullptr, 0);
            }
            break;
        case 0x08: // tests
            if (packet_size == 1) { // Check packet size
                uint32_t t[] = {200};
                state = TESTS;
                writeSerial(true, (uint8_t*)t, sizeof(t));
            }
            break;
        default:
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

  Serial.write(0x00); // Footer
  Serial.flush();
}

int Raspy::get_State(){
    return state;
}
