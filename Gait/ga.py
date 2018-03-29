import random
import ConfigParser
import rospy
import json
from copy import deepcopy

"""
Things need to discuss:

min-max angles for servos
ROS topic and message details for sending the gait
Test procedure for fitness evaluation (move straight line, whatever)
Judging stability
"""

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
        self.states = []

        for i in range(6):
            for j in range(3):

                # min/max angles for each servo can be different, according to config
                # i*3 + j gets us a value from [0, 17] to access a value for our 18 servos
                self.states.append(
                    Leg(
                        [random.rantint(angle_min[i*3 + j], angle_max[i*3 + j])]
                    )
                )

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
        
    def check_stability(self):
        return True

    def __eq__(self, other):
        return hasattr(other, 'fitness') and self.fitness == other.fitness

    def __lt__(self, other):
        return hasattr(other, 'fitness') and self.fitness < other.fitness

class MCU:
    def __init__(self):
        self.pi = pigpio.pi()
        
        # Open SPI 0 (idk why, thats the one) at 115200 baud
        self.spi = pi.spi_open(0, 115200)
        
    def send_gait_mcu(self, GeneratorExit):
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
        for step in gait.steps:
            for leg in step.legs:
                for state in leg.states:
                    (count, rx_data) = self.pi.spi_xfer(self.spi, state)

class Leg:
    def __init__(self, legs)
        self.legs = legs

class Config:
    def __init__(self, filename):
        # Read the config file to have values for:
        # Step count
        # Angle values for intialization
        # Generations
        cfgParser = ConfigParser.RawConfigParser()
        cfgParser.read(filename)

        self.num_generations = cfgParser.getint('GA', 'num_generations')
        self.population_size = cfgParser.getint('GA', 'population_size')
        self.num_runs = cfgParser.getint('GA', 'num_runs')
        self.crossover_probability = cfgParser.getfloat('GA', 'crossover_probability')
        self.crossover_leg_probability = cfgParser.getfloat('GA', 'crossover_leg_probability')
        self.mutation_probability = cfgParser.getfloat('GA', 'mutation_probability')
        self.mutation_servo_probability = cfgParser.getfloat('GA', 'mutation_servo_probability')
        self.tournament_size = cfgParser.getint('GA', 'tournament_size')
        self.mutation_change = cfgParser.getint('GA', 'mutation_change')

        self.num_steps = cfgParser.getint('Population', 'num_steps')
        self.angle_min = json.loads(cfgParser.get('Population', 'angle_min'))
        self.angle_max = json.loads(cfgParser.get('Population', 'angle_max'))

        self.node_name = cfgParser.get('ROS', 'node_name')

class GA:
    def __init__(self, config):
        self.config = config
        self.num_generations = config.num_generations
        self.num_runs = config.num_runs
        self.population_size = config.population_size
        
    def generate_population(self):
        self.population = [Gait(num_steps=self.config.num_steps, angle_min=self.config.angle_min, angle_max=self.config.angle_max) 
            for i in range(self.population_size)]

    # Write the state of the GA to a file to continue later
    def save_checkpoint(self):
        pass

    def crossover(self, gait1, gait2):
        stable = False

        while not stable:
             for step in gait1

    def mutate(self, gait):
        slicing_point = random.randint(0, len(gait.steps) - 1)
        stable = False
        
        new_gait = deepcopy(gait)

        while not stable:
            # Go through the steps after and including the slicing point (so that step 0 can be chosen)
            for step in new_gait.steps[slicing_point:]:
                for leg in step.legs:
                    if random.random() <= self.config.crossover_leg_probability:
                        for state in leg.states:
                            state += self.config.mutation_change

            stable = new_gait.check_stability()

        new_gait.fitness = self.evaluate_individual(gait)

        return max(gait, new_gait)

    # Selects k random individuals and returns the most fit one
    def tournament_selection(self, population, tournament_size):
        index1 = random.randint(0, self.population_size - 1)
        tournament_players = population[index1]

        for i in range(tournament_size):
            opponent = index1
            
            while opponent == index1:
                opponent = random.randint(0, self.population_size - 1)
            
            tournament_players.append(population[opponent])
        
        tournament_players = sorted(tournament_players)

        return tournament_players[-1]

    def run(self, config):
        # Runs
        for run in range(self.num_runs):
            population = self.generate_population(config)
            best_gait = None

            for generation in range(self.num_generations):
                # Evaluate the individuals somehow, probably going to have to run them through ROS
                
                for individual in population:
                    individual.fitness = self.evaluate_individual(individual)

                new_population = []

                # Crossover time
                for gait in population:
                    if best_gait == None or gait.fitness > best_gait.fitness:
                        best_gait = gait

                    gait1 = self.tournament_selection(population, config.tournament_size)
                    gait2 = self.tournament_selection(population, config.tournament_size)

                    if random.random <= config.crossover_probability:
                        child = self.crossover(gait1, gait2)
                        new_population.append(child)
                        
                for gait in population:
                    if random.random <= config.mutation_probability:
                        self.mutate(gait, config.mutation_change)
                
                sorted_population = sorted(population)
                new_population += sorted_population[:self.population_size-len(new_population)]
                # Should I reshuffle this array since most of it is sorted at this point? Add randomness to selection

    def evaluate_individual(self, gait):
        under_30 = 0
        fitness = 0
        for step in gait.steps:
            for leg in step.legs:
                for state in leg.states:
                    if state < under_30:
                        fitness += 1

        return fitness


if __name__ == '__main__':
    config = Config('config.cfg')

    rospy.init_node(config.node_name)

    ga = GA(config)
    ga.run(config)
    