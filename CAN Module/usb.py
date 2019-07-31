import serial

arduino = serial.Serial('/dev/ttyUSB0',19200, timeout=.1)
arduino.flushInput()
while True:
  try:
     data = arduino.readline()
     if data:
        parsed = data.split('\t')
        parsed = list(map(int,parsed[:-1]))
        print(parsed)
  except:
    print('Error')
