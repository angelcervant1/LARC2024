arduino.connect()
    # arduino.open()
    while True:
        print(arduino.sendLocation(0))
        print(arduino.sendLocation(1))
        time.sleep(1)