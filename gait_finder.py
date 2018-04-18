from __future__ import print_function
import struct
import pigpio
import sys
print(sys.argv)
if len(sys.argv) != 4:
        print('Incorrect args, give a servo id and pos')
pi = pigpio.pi()
spi = pi.spi_open(0, 115200, 0)

cmd = int(sys.argv[1])
servo_id = int(sys.argv[2])
pos = int(sys.argv[3])
message = servo_id << 10 | pos
print('Sending CMD {} position {} to Servo {} as message: {}'.format(cmd, pos, servo_id, bin(message)))
fragmented_states = [cmd] + list(struct.unpack('2B', struct.pack('<h', message)))
print(fragmented_states)
pi.spi_xfer(spi,fragmented_states)

pi.spi_close(spi)
