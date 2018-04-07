import pigpio
import struct
from time import sleep

pi = pigpio.pi()
spi = pi.spi_open(0, 115200)
servo1 = 420
servo2 = 200
servo3 = 117
command = 2
states = (command << 30) | (servo3 << 20) | (servo2 << 10) | servo1

print bin(states)
fragmented_states = struct.unpack('4B', struct.pack('<I', states))
print bin(states)
print ''

# thrown away
#(count, rx_data) = pi.spi_xfer(spi, 'a')

for state in fragmented_states:
    print(bin(state))
    (count, rx_data) = pi.spi_xfer(spi, [state])
    print count,rx_data
    sleep(1)

pi.spi_close(spi)