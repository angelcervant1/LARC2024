import communication
import time

if __name__ == "__main__":
    arduino = communication.Arduino()
    arduino.connect()
    # arduino.open()
    print(arduino.cube_found())
    time.sleep(1)