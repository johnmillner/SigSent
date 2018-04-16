#!/usr/bin/python
import rospy
import pigpio
from std_msgs.msg import Int8
import roslib
from sigsent.msg import Drive

roslib.load_manifest('sigsent')

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
        
        self.mode = 0

        self.pi = pigpio.pi()
        self.spi_bus = 1
        self.spi_baud = 115200
        self.spi = self.pi.spi_open(self.spi_bus, self.spi_baud)

        self.message_gen = Message()

        # Subscribers and their SPI callbacks here
        self.mode_sub = rospy.Subscriber('mode', Int8, self.mode_cb, queue_size=1)
        self.walking_sub = rospy.Subscriber('walk', Int8, self.walk_cb, queue_size=1)
        self.drive_sub = rospy.Subscriber('drive', Drive, self.drive_cb, queue_size=1)
        
        self.direction_list = [False] * 4

        self.current_msg = None

    def mode_cb(self, data):
        if data.data == 0:
            self.mode = 0
            self.pi.spi_xfer(self.spi, self.message_gen.create_mode_change_message(driving=True))
        elif data.data == 1:
            self.mode = 1
            self.pi.spi_xfer(self.spi, self.message_gen.create_mode_change_message(walking=True))
        
    def walk_cb(self, data):
        if data.data < 0 or data.data > 4:
            return

        if data.data == 3:
            self.current_msg = None

        directions = list(self.direction_list)
        directions[data.data] = True
        self.current_msg = self.message_gen.create_walking_message(fwd=directions[0], left=directions[1], right=directions[2])
        
    def drive_cb(self, data):
        if data.direction.data < 0 or data.direction.data > 3:
            return

        if data.speed.data == 0:
            self.current_msg = None

        directions = list(self.direction_list)
        directions[data.direction.data] = True

        self.current_msg = self.message_gen.create_esc_message(fwd=directions[0], left=directions[1], right=directions[2], back=directions[3], speed=data.speed.data)
            
if __name__ == '__main__':
    try:
        spi = Spi()

        while not rospy.is_shutdown():
            # Driving mode
            if spi.mode == 0 and spi.current_msg != None:
                self.pi.spi_xfer(self.spi, self.current_msg)

            # Walking mode
            elif spi.mode == 1  and spi.current_msg != None:
                self.pi.spi_xfer(self.spi, self.current_msg)        

            self.rate.sleep()
    except rospy.ROSInterruptException:
        pass