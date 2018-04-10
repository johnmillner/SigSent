#!/usr/bin/python
import rospy
import pigpio
from std_msgs.msg import Int8
class Message:
    """
    Functions called by the user return a list of bytes containing message header and needed content messages.
    The SPI library from pigpio take a list of bytes and send them, so this makes life easy!
    """

    def __init__(self):

        # Header message bits define what message is coming next so that the MCU
        # knows how to parse it
        self.mode_change = 0b00110000
        self.walking_move = 0b00001100
        self.esc = 0b00000011

        # Directions define how the walking robot should move through gait and define
        # what direction the ESCs need to send the motors in
        self.fwd = 0b00000000
        self.left = 0b00001111
        self.right = 0b11110000
        self.back = 0b11111111

        # Mode type comes after mode_change header and tells the hexapod to switch to
        # one of the following mobility mechanism
        self.driving_mode = 0b00001111
        self.walking_mode = 0b11110000

    # Creates a header message with a given type provided by the user
    def _create_header(self, mode_change=False, walking_move=False, esc=False):
        header = 0b00000000

        if mode_change:
            header |= self.mode_change
        elif walking_move:
            header |= self.walking_move
        elif esc:
            header |= self.esc

        return header
        
    def create_walking_message(self, fwd=False, left=False, right=False):
        messages = []
        messages.append(self._create_header(walking_move=True))
        messages.append(self.create_direction_message(fwd=fwd, left=left, right=right))

        return messages
    
    def create_mode_change_message(self, driving=False, walking=False):
        messages = []
        messages.append(self._create_header(mode_change=True))
        message = 0b00000000
        if driving:
            message |= self.driving_mode
        elif walking:
            message |= self.walking_mode
        messages.append(message)

        return messages

    def create_direction_message(self, fwd=False, left=False, right=False, back=False):
        message = 0b00000000
        if fwd:
            message |= self.fwd
        elif left:
            message |= self.left
        elif right:
            message |= self.right
        elif back:
            message |= self.back
        
        return message

    def create_esc_message(self, fwd=False, left=False, right=False, back=False, speed=0):
        messages = []
        messages.append(self._create_header(esc=True))

        # Speed can only be stored in one byte. I check the bit_length rather than the int
        # value specifically because Python has no distinction between signed/unsigned ints
        if speed.bit_length() > 8:
            return None

        messages.append(self.create_direction_message(fwd=fwd, left=left, right=right, back=back))
        messages.append(speed)
        
        return messages

class Spi:
    """
    Everything that needs to be passed to the MCU over SPI should have a ROS
    subscriber defined here where in the callback, the spi transfer takes place
    """
    def __init__(self):
        rospy.init_node('spi_to_mcu')
        
        self.pi = pigpio.pi()
        self.spi_bus = 1
        self.spi_baud = 115200

        # Subscribers here
        self.mode_sub = rospy.Subscriber('mode', Int8, self.mode_cb)
        
        self.message_gen = Message()

    def mode_cb(self, data):
        spi = self.pi.spi_open(self.spi_bus, self.spi_baud)

        if data.data == 0:
            self.pi.spi_xfer(spi, self.message_gen.create_mode_change_message(driving=True))
        elif data.data == 1:
            self.pi.spi_xfer(spi, self.message_gen.create_mode_change_message(walking=True))
        
        self.pi.spi_close(spi)

if __name__ == '__main__':
    try:
        spi = Spi()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass