import serial

IMUport     = '/dev/ttyUSB0'
IMUbaud     = 115200
IMUparity   = serial.PARITY_NONE
IMUstopBits = serial.STOPBITS_ONE
IMUbyteSize = serial.EIGHTBITS
IMUtimeout  = 0

ser = serial.Serial('/dev/ttyUSB0', IMUbaud, timeout = IMUtimeout) as ser:
    line = ser.readline()
    

print(line)
