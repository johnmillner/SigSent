from __future__ import print_function

import pigpio
from pi_spi_modules import Message
from time import sleep

"""
main to test the message sending. These should be called from a ROS py file
that needs to...

Change mode: Button in GUI, publishes message
TeleOp/Move base: publishes to topics. Listen to them and send the appropriate SPI
"""
if __name__ == '__main__':

    pi = pigpio.pi()
    spi = pi.spi_open(0, 115200)
    tests = []
    test_message = Message()
    
    tests.append(test_message.create_walking_message(fwd=True))
    tests.append(test_message.create_walking_message(left=True))
    tests.append(test_message.create_walking_message(right=True))

    tests.append(test_message.create_esc_message(left=True, speed=10))
    tests.append(test_message.create_esc_message(right=True, speed=127))

    tests.append(test_message.create_mode_change_message(driving=True))
    tests.append(test_message.create_mode_change_message(walking=True))

    for test in tests:
        print('Sending test: {}'.format(test))

        pi.spi_xfer(spi, test)
        sleep(1)