import random
import ConfigParser
import json
import itertools
import pigpio
import math
from copy import deepcopy
from functools import total_ordering
from time import sleep

"""
Things need to discuss:

min-max angles for servos
ROS topic and message details for sending the gait
Test procedure for fitness evaluation (move straight line, whatever)
Judging stability
"""

class Leg:
    def __init__(self, states):
        self.states = list(states)

    def __repr__(self):
        return 'Leg: {}'.format(self.states)

class Step:
    def __init__(self, legs=None, angle_min=None, angle_max=None):
        if legs == None:
            self.generate_legs(angle_min, angle_max)
        else:
            self.legs = legs

    def generate_legs(self, angle_min, angle_max):
        self.legs = []
        
        for i in range(6):
            states = []
            for j in range(3):
                # min/max angles for each servo can be different, according to config
                # i*3 + j gets us a value from [0, 17] to access a value for our 18 servos
                states.append(random.randint(angle_min[i*3 + j], angle_max[i*3 + j]))
            l = Leg(states)
            
            self.legs.append(Leg(states))

    def __repr__(self):
        return 'Step\n{}\n'.format(', '.join([str(leg) for leg in self.legs]))

@total_ordering
class Gait:
    def __init__(self, steps=None, num_steps=None, angle_min=None, angle_max=None):
        if steps == None:
            self.generate_steps(num_steps, angle_min, angle_max)
        else:
            self.steps = steps

        self.fitness = 0

    def generate_steps(self, num_steps, angle_min, angle_max):
        self.steps = [Step(angle_min=angle_min, angle_max=angle_max) for i in range(num_steps)]
        
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
        self.spi = self.pi.spi_open(0, 115200)
       
    def send_gait_mcu(self, gait):
        # Send each servo state for all possible 216 states (3 servos * 6 legs * 12 steps)
        for step in self.gait.steps:
            for leg in step.legs:
                for state in leg.states:
                    (count, rx_data) = self.pi.spi_xfer(self.spi, state)

    def degrees_to_servo(self, deg):
        return math.ceil(val = deg/.293)

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
    """
    Things to save:

    Run
    Generation
    Population
    
    """
    def save_checkpoint(self):
        pass

    def crossover(self, gait1, gait2):
        stable = False

        legs1 = []
        legs2 = []

        # Technically with crossover here, we should have two children since swapping
        # from parent1 or parent2 provides two unique combinations as a result
        # but the paper only shows 1 offspring, so we will arbitrarily choose the 
        # one where we take the modified gait1
        new_gait = deepcopy(gait1)

        # Zips the two steps lists into a tuple of each step object
        # Appends the legs from each step object to a list of the total legs
        for step1,step2 in zip(new_gait.steps, gait2.steps):
            legs1 += step1.legs
            legs2 += step2.legs

        while not stable:
            # Gets the cartesian product of the two leg lists for all
            # possible pairwise combinations of legs
            for leg1,leg2 in list(itertools.product(legs1, legs2)):

                # If random value less than probability, swap the leg state values
                # We don't have to modify leg2 since we said to arbitrarily choose the first gait
                if random.random() < self.config.crossover_leg_probability:
                    leg1.states = leg2.states
            
            stable = new_gait.check_stability()

        return max(gait1, gait2, new_gait)
            

    def mutate(self, gait, config):
        slicing_point = random.randint(0, len(gait.steps) - 1)
        
        stable = False
        changed_one = False

        new_gait = deepcopy(gait)

        while not stable:
            # Go through the steps after and including the slicing point (so that step 0 can be chosen)
            for step in new_gait.steps[slicing_point:]:
                for i,leg in enumerate(step.legs):
                    for j,state in enumerate(leg.states):
                        if random.random() <= self.config.mutation_servo_probability:
                            changed_one = True
                            if random.random() <= 0.49:
                                state += self.config.mutation_change
                            else:
                                state -= self.config.mutation_change
                            
                            # Restrict state to be within the angle_min and angle_max
                            # Overflow into the correct value
                            if state > config.angle_max[i*3 + j]:
                                state = config.angle_min[i*3 + j] + (state - config.angle_max[i*3 + j])
                            elif state < config.angle_min[i*3 + j]:
                                state = config.angle_max[i*3 + j] - (config.angle_min[i*3 + j] - val)

            if changed_one == False:
                gait.steps[random.randint(0, 11)].legs[random.randint(0, 5)].states[random.randint(0,2)] -= self.config.mutation_change

            stable = new_gait.check_stability()

        new_gait.fitness = self.evaluate_individual(gait)

        return max(gait, new_gait)

    # Selects k random individuals and returns the most fit one
    def tournament_selection(self, population, tournament_size):
        index1 = random.randint(0, self.population_size - 1)
        tournament_players = [population[index1]]

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
            best_gait = None
            self.generate_population()
            
            for generation in range(self.num_generations):
                # Evaluate the individuals somehow, probably going to have to run them through ROS
                
                for individual in self.population:
                    individual.fitness = self.evaluate_individual(individual)
                    
                new_population = []
                # Crossover time
                for gait in self.population:
                    if best_gait == None or gait.fitness > best_gait.fitness:
                        best_gait = gait

                    gait1 = self.tournament_selection(self.population, config.tournament_size)
                    gait2 = self.tournament_selection(self.population, config.tournament_size)

                    if random.random() <= config.crossover_probability:
                        new_population.append(self.crossover(gait1, gait2))
                        
                for gait in self.population:
                    if random.random() <= config.mutation_probability:
                        new_population.append(self.mutate(gait, config))
                
                sorted_population = sorted(self.population, reverse=True)
                new_population += sorted_population[:self.population_size-len(new_population)]

                self.population = new_population

                print('Generation {}'.format(generation))
                print('Average: {}'.format(self.average_fitness()))
                print('Best: {}\n\n'.format(best_gait.fitness))
            print(best_gait.steps)
                # Should I reshuffle this array since most of it is sorted at this point? Add randomness to selection

    def average_fitness(self):
        avg = 0
        for gait in self.population:
            avg += gait.fitness
        
        return avg / len(self.population)
    
    def evaluate_individual(self, gait):
        fitness = 0
        
        # silly example, GA looks for gaits with angles under 30 degrees
        for step in gait.steps:
            for leg in step.legs:
                for state in leg.states:
                    if state < 30:
                        fitness += 1

        return fitness


if __name__ == '__main__':
    config = Config('config.cfg')

    ga = GA(config)
    ga.run(config)
    