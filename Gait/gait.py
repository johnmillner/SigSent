import rospy
import pigpio
from functools import total_ordering
from geometry_msgs.msg import Twist
# Take in /cmd/vel ROS topic and translate it into moving SigSent

class Leg:
    def __init__(self, states):
        self.states = states

class Step:
    def __init__(self, legs=None, angle_min=None, angle_max=None):
        if legs == None:
            self.generate_legs(angle_min, angle_max)
        else:
            self.legs = legs

    def generate_legs(self, angle_min, angle_max):
        self.states = [Leg([random.randint(angle_min, angle_max) for i in range(3)]) for j in range(6)]

@total_ordering
class Gait:
    def __init__(self, steps=None, num_steps=None, angle_min=None, angle_max=None):
        if steps == None:
            self.generate_steps(num_steps, angle_min, angle_max)
        else:
            self.steps = steps

        self.fitness = 0

    def generate_steps(self, num_steps, angle_min, angle_max):
        self.steps = [Step(angle_min, angle_max) for i in range(num_steps)]
        
    def __eq__(self, other):
        return hasattr(other, 'fitness') and self.fitness == other.fitness

    def __lt__(self, other):
        return hasattr(other, 'fitness') and self.fitness < other.fitness

class SigSent:
    def __init__(self, gait=None):
        self.gait = gait
        self.vel_sub = rospy.Subscriber('cmd_vel', Twist,  self.move, queue_size=1)
        self.pi = pigpio.pi()
        
        # Open SPI 0 (idk why, thats the one) at 115200 baud
        self.spi = pi.spi_open(0, 115200)
        
    def send_gait_mcu(self):
        """
        Sample Arduino SPI code RX

        char step = 0;
        char leg = 0;
        char servo = 0;

        int[][][] gait = new int[12][6][3];

        // SPI interrupt routine
        ISR (SPI_STC_vect)
        {
            // grab byte from SPI Data Register
            byte data = SPDR;
            
            char num_servos_set = (step * servo * leg + leg * servo + servo);

            // Add to gait if we haven't set everything
            if (num_servos_set < sizeof gait)
            {
                gait[step][leg][servo] = data;

                servo++;

                if (servo > 2)
                {
                    servo = 0;
                    leg++;

                    if (leg > 5)
                    {
                        step++;
                    }
                }
            }
        }
        """
        
        # Send each servo state for all possible 216 states (3 servos * 6 legs * 12 steps)
        for step in self.gait.steps:
            for leg in step.legs:
                for state in leg.states:
                    (count, rx_data) = self.pi.spi_xfer(self.spi, state)



class Leg:
    def __init__(self, legs)
        self.legs = legs

class Servo:
    def __init__(self, address):
        self.address = address
    
if __name__ == '__main__':
    