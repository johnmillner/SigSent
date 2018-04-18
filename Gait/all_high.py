import pigpio
import struct
from time import sleep

pi = pigpio.pi()
pi.write(17, 1)
