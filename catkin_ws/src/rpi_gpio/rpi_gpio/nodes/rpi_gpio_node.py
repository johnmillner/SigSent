#!/usr/bin/env python

import os, time

import rospy

import wiringpi

IN = 'in'
OUT = 'out'

from rpi_gpio.srv import *
from rpi_gpio import msg as msgs

#http://elinux.org/RPi_Low-level_peripherals#Model_A.2B.2C_B.2B_and_B2
RPI2_GPIO_PINS = (
    0,#ID_SD
    1,#ID_SC
    2,#SDA1
    3,#SCL2
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,#TXD0
    15,#RXD0
    16,
    17,
    18,
    19,
    20,#MOSI
    21,#SCLK
    22,
    23,
    24,
    25,
    26,
    27,
)

class RPiGPIO():
    
    def __init__(self):
        
        rospy.init_node('rpi_gpio', log_level=rospy.DEBUG)
        
        # Pin numbers are the GPIO# value, not the literal pin number.
        # Setting direction here is necessary because wiringpi doesn't support setMode() at time of writing.
        self.directions = rospy.get_param("~directions", {})
        
        # These states will be set to pins right before the node shuts down.
        self.shutdown_states = rospy.get_param("~shutdown_states", {})
        
        # Set default states.
        self.states = rospy.get_param("~states", {})
        
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)
        
        for pin, direction in self.directions.items():
            if not isinstance(pin, int):
                del self.directions[pin]
            pin = int(pin)
            self.directions[pin] = direction
            # May give the error:
            # bash: echo: write error: Device or resource busy
            # if the pin was left exported
            assert pin in RPI2_GPIO_PINS, 'Invalid pin: %s' % pin
            assert direction in (IN, OUT), 'Invalid direction for pin %s: %s' % (pin, direction)
            
            self.export_pin(pin, direction)
        
        wiringpi.wiringPiSetupSys()
        
        for pin, state in self.states.items():
            if not isinstance(pin, int):
                del self.states[pin]
            pin = int(pin)
            self.states[pin] = state
            msg = DigitalWrite()
            msg.pin = int(pin)
            msg.state = state
            self._set_pin_handler(msg)
    
        for pin, state in self.shutdown_states.items():
            if not isinstance(pin, int):
                del self.shutdown_states[pin]
            pin = int(pin)
            self.shutdown_states[pin] = state
            assert pin in RPI2_GPIO_PINS, 'Invalid pin: %s' % pin
            assert state in (0, 1), 'Invalid shutdown state for pin %s: %s' % (pin, state)
    
        # Define publishers and services.
        #self.get_pin_pub = rospy.Publisher('~get_pin', msgs.DigitalRead)
        self.pin_change_pub = rospy.Publisher('~pin_change', msgs.DigitalChange, queue_size=10)
        rospy.Service('~set_pin', DigitalWrite, self._set_pin_handler)
        
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():

            # Publish all sensor values on a single topic for convenience
            now = rospy.Time.now()

            #TODO: read all readable-pins and publish an event on change

            r.sleep()
 
    def export_pin(self, pin, direction, max_retries=10):
        if not os.path.isfile('/sys/class/gpio/gpio{pin}'.format(pin=pin)):
            cmd = 'echo {pin} > /sys/class/gpio/export'.format(pin=pin)
            print cmd
            os.system(cmd)
        success = False
        for retry in xrange(max_retries):
            cmd = 'echo {direction} > /sys/class/gpio/gpio{pin}/direction'.format(pin=pin, direction=direction)
            print cmd
            ret = os.system(cmd)
            if ret:
                time.sleep(0.1)
            else:
                success = True
                break
        assert success, 'Unable to expot pin %s as %s: %s' % (pin, direction, ret)
 
    def _set_pin_handler(self, msg):
        assert msg.pin in RPI2_GPIO_PINS, 'Invalid pin: %s' % msg.pin
        assert msg.state in (True, False, 0, 1), 'Invalid state: %s' % msg.state
        assert msg.pin in self.directions, 'Pin %s has not been exported.' % msg.pin
        assert self.directions[msg.pin] == OUT, 'Pin %s is not an output.' % msg.pin
        wiringpi.digitalWrite(msg.pin, msg.state)
        
        old_state = self.states.get(msg.pin, None)
        self.states[msg.pin] = msg.state
        if old_state != msg.state:
            new_msg = msgs.DigitalChange()
            new_msg.pin = msg.pin
            new_msg.state = msg.state
            self.pin_change_pub.publish(new_msg)
        
        return DigitalWriteResponse()
 
    def shutdown(self):
        
        # Reset pin state.
        rospy.loginfo("Reseting pin states...")
        for pin, state in self.shutdown_states.iteritems():
            msg = DigitalWrite()
            msg.pin = int(pin)
            msg.state = state
            self._set_pin_handler(msg)
        
        # Reset pin exports.
        rospy.loginfo("Unexporting pins...")
        for pin, direction in self.directions.iteritems():
            cmd = 'cd /sys/class/gpio; echo {pin} > unexport'.format(pin=pin)
            print cmd
            os.system(cmd)
        
if __name__ == '__main__':
    myArduino = RPiGPIO()
