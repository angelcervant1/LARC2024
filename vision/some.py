import communication
import time

if __name__ == "__main__":
    arduino = communication.Arduino()
    arduino.connect()
    # arduino.open()
    while True:
        print(arduino.sendLocation(0))
        time.sleep(1)