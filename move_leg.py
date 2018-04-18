from __future__ import print_function
import struct
import pigpio
import sys

def move(servo_id, pos):
	cmd = 20
	pi = pigpio.pi()
	spi = pi.spi_open(0, 115200, 0)
	message = servo_id << 10 | pos
	fragmented_states = [cmd] + list(struct.unpack('2B', struct.pack('<H', message)))
	pi.spi_xfer(spi,fragmented_states)
	pi.spi_close(spi)

if __name__ == '__main__':
	move(int(sys.argv[1]), int(sys.argv[2]))
