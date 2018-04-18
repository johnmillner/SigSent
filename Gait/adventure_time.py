import pigpio

pi = pigpio.pi()
spi = pi.spi_open(1, 115200)

count, rx = pi.spi_xfer(spi, [42, 69, 0])
for thing in rx:
	print thing

pi.spi_close(spi)
