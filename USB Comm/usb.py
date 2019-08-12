
import serial

#Serial(name of port, baud rate, timeout)
arduino = serial.Serial('/dev/ttyUSB0',19200, timeout=.1)	
arduino.flushInput()
while True:
  try:
     data = arduino.readline()
     #print(data)
     if data:
        parsed = data.split('\t')
        parsed = list(map(float,parsed[:-1]))
        print(parsed)
  except:
    print('Error')
