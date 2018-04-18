import pigpio
import struct
from time import sleep

pi = pigpio.pi()
pi.set_mode(17, pigpio.OUTPUT)
pi.write(17, 0)
