import communication

if __name__ == "__main__":
    arduino = communication.Arduino()
    arduino.connect()
    # arduino.open()
    print(arduino.get_baud())