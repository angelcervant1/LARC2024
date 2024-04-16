import communication
import time

if __name__ == "__main__":
    arduino = communication.Arduino()
    arduino.connect()
    # arduino.open()
    while True:
        print(arduino.cube_found(1.00, True))
        time.sleep(1)