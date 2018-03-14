import random
import ConfigParser
import rospy
from functools import total_ordering

"""
Things need to discuss:

min-max angles for servos
ROS topic and message details for sending the gait
Test procedure for fitness evaluation (move straight line, whatever)
Judging stability
"""

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
        self.crossover_probability = cfgParser.getint('GA', 'crossover_probability')
        self.crossover_leg_probability = cfgParser.getint('GA', 'crossover_leg_probability')
        self.mutation_probability = cfgParser.getint('GA', 'mutation_probability')
        self.tournament_size = cfgParser.getint('GA', 'tournament_size')
        self.mutation_change = cfgParser.getint('GA', 'mutation_change')

        self.num_steps = cfgParser.getint('Population', 'num_steps')
        self.angle_min = cfgParser.getint('Population', 'angle_min')
        self.angle_max = cfgParser.getint('Population', 'angle_max')

        self.node_name = cfgParser.get('ROS', 'node_name')

class Step:
    def __init__(self, states=None, angle_min=None, angle_max=None):
        if states == None:
            self.generate_states(angle_min, angle_max)
        else:
            self.states = states

    def generate_states(self, angle_min, angle_max):
        self.states = [random.randint(angle_min, angle_max) for i in range(3)]

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


class GA:
    def __init__(self, config):
        self.num_generations = config.num_generations
        self.num_runs = config.num_runs
        self.population_size = config.population_size

    def generate_population(self, config):
        self.population = [Gait(num_steps=config.num_steps, angle_min=config.angle_min, angle_max=config.angle_max) 
            for i in range(self.population_size)]

    # Write the state of the GA to a file to continue later
    def save_checkpoint(self):
        pass

    def crossover(self, gait1, gait2):
        stable = False

        # TODO (rwales): Finish Xover process
        while not stable:
             pass

    def mutate(self, gait, mutation_change):
        pass

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



if __name__ == '__main__':
    config = Config('config.cfg')

    rospy.init_node(config.node_name)

    ga = GA(config)
    ga.run(config)
    