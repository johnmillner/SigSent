#!/usr/bin/env python

from __future__ import print_function
import os
import neat
import visualize
import rospy
import pickle
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu

class NEATTrainer:
    def __init__(self):
        
        # Setup NEAT
        local_dir = os.path.dirname(__file__)
        config_path = os.path.join(local_dir, 'config-ann')

        config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation,
                     config_path)

        population = neat.Population(config)

        population.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        population.add_reporter(stats)
        population.add_reporter(neat.Checkpointer(5)) # Saves every 5 generations as checkpoints to fall back on

        # Pass the eval_genomes function to the population object
        # 'eval_genomes' will be called on each individual in the population to determine their
        # fitness each generation
        # 300 is the number of generations to run. This should be configured manually and tweaked for best results
        most_fit_genome = population.run(self.eval_genomes, 300)
        most_fit_network = neat.nn.FeedForwardNetwork.create(most_fit_genome, config)

        with open('best_net.ann', 'wb') as net_fp:
            pickle.dump(most_fit_network, net_fp)

        visualize.draw_net(config, most_fit_genome, True)
        visualize.plot_stats(stats, ylog=False, view=True)
        visualize.plot_species(stats, view=True)
        
    def init_training_set(self, filename):
        self.training_set = {}

        with open(filename, 'r') as training_fp:
            self.training_set = json.load(training_fp)
    
    def eval_genomes(self, genomes, config):
        for genome_id, genome in genomes:
            print('evaluating genome {}'.format(genome_id))
            
            net = neat.nn.FeedForwardNetwork.create(genome, config)
            fitness = 0
            
            for imu_data in self.training_set:
                inputs = []
                for imu_state in imu_data:
                    inputs += list(imu_state)
                
                output = 1 if net.activate(inputs) >= 0.5 else 0

                if output == self.training_set[imu_data]:
                    fitness += 1
            
            genome.fitness = fitness
        
if __name__ == '__main__':
    neatNode = NEATTrainer()