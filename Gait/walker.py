import pigpio

pi = pigpio.pi()
spi = pi.spi_open(1, 115200, 0)

try:
	while True:
		command = raw_input()
		if command == 'w':
			command = 10
			pi.spi_xfer(spi, [10])
		elif command == 'd':
			pi.spi_xfer(spi, [20])
except Exception as e:
	pi.spi_close(spi)
