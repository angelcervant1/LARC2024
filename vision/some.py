import communication

if __name__ == "__main__":
    arduino = communication.Arduino()
    arduino.connect()
    print(arduino.test(int(1)))