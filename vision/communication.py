#!/usr/bin/env python3
import serial
import Constants
import time

class Arduino(): 
    def __init__(self):
        self.arduino = serial.Serial(Constants.serial_port, Constants.baud_rate, timeout=Constants.timeout)
        pass

    def read(self):
        data = self.arduino.readline().decode()
        print(data)
        return data
    
    def write(self, msg):
        send = str(msg)
        send += "\n"
        self.arduino.write(bytes(send, 'utf-8')) 