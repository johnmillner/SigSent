import serial

IMUport     = '/dev/ttyUSB0'
IMUbaud     = 115200
IMUparity   = serial.PARITY_NONE
IMUstopBits = serial.STOPBITS_ONE
IMUbyteSize = serial.EIGHTBITS
IMUtimeout  = 0

ser = serial.Serial( \
    port = IMUport, baudrate = IMUbaud, parity = IMUparity, \
    stopbits = IMUstopBits, bytesize = IMUbyteSize, timeout = IMUtimeout )
    
print("connected to: " + ser.portstr)

while True:
    line = ser.readline();
    if line:
        print(line),
    ser.close()
