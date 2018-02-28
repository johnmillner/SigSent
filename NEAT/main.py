#!/usr/bin/env python

from __future__ import print_function
import os
import neat
import visualize
import rospy
import message_filters
from std_msgs.msg import Bool, Image, Imu, LaserScan

class NEATNode:
    def __init__(self):
    
        # Setup the ROS sensor subscriptions
        self.sensor_data = {}
        
        rospy.init_node('neat', anonymous=True)
        
        mobility_pub = rospy.Publisher('mobility_type', Bool)
        imu_sub = rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.callback, ('imu'))
        cam_sub = rospy.Subscriber('/cam1/raw_image', Image, self.callback, ('camera'))
        lidar_sub = rospy.Subscriber('/scan', LaserScan, self.callback, ('lidar'))
        
        # Setup NEAT
        local_dir = os.path.dirname(__file__)
        config_path = os.path.join(local_dir, 'config-ann')

        config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation,
                     config_file)

        population = neat.Population(config)

        population.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        population.add_reporter(stats)
        population.add_reporter(neat.CheckPointer(5)) # Saves every 5 generations as checkpoints to fall back on

        # Pass the eval_genomes function to the population object
        # 'eval_genomes' will be called on each individual in the population to determine their
        # fitness each generation
        # 300 is the number of generations to run. This should be configured manually and tweaked for best results
        most_fit_genome = population.run(self.eval_genomes, 300)
        most_fit_network = neat.nn.FeedForwardNetwork.create(most_fit_genome, config)

        # Should probably do a test run with the winning genome's network here to see how well it learned

        # Write the genome to a file maybe to save it for later
        # That way when we are not in a learning period, we can just load up the genome,
        # create the ANN from it, and use it to modify the mobility mechanism

        # Outputting some graphs with the network, and some graphs on fitness and speciation
        visualize.draw_net(config, most_fit_genome, True)
        visualize.plot_stats(stats, ylog=False, view=True)
        visualize.plot_species(stats, view=True)
        
    def callback(self, data, args):
        self.sensor_data[args[0]] = data
    
    def eval_genomes(genomes, config):
        for genome_id, genome in genomes:
            # Take the ANN from the population and pass it somewhere to be used
            net = neat.nn.FeedForwardNetwork.create(genome, config)
            
            """
            Notes for ideas on how to use this NET...

            1) Use this net to calculate an output on the mobility mechanism state
            2) Send that binary value (0/1) to the ROS topic
            3) Have a generous delay to avoid constant alternations of states
            4) Measure movement statistics during this waiting period where the robot's general stability
               is used to measure fitness
            5) Set the fitness of this network based on the measured stats
            6) Move on to the next network in the population and do it again
            """

            inputs = []

            # Poll the ROS topic that contains the Terrain Classification
            # inputs.append(...)
            
            # Topics:
            # IMU: /mobile_base/sensors/imu_data
            # Camera image: /cam1/image_raw
            # LIDAR: /scan        

            
            
            
            # Get the output boolean
            output = True if net.activate() >= 0.5 else False

            # Publish the True/False bool
            pub.publish(output)        
                    
                    
            ############################
            #                          #
            # Time to test out the ANN #
            #                          #
            ############################
            cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
            r = rospy.Rate(10);
            
            move_cmd = Twist()
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0

            while not rospy.is_shutdown():
                cmd_vel.publish(move_cmd)
                
                # Gather some stats on how well we are moving for fitness
                # calculate distance moved
                # get the IMU data uniformity
                
                r.sleep()
            
            
            # Poll the ROS topic for the IMU data
            # inputs.append(...)


            # Loop here to gather stats on the IMU for robot stability
            # and distance traveled/time spent attempting to move forward
            # Should those stats be gathered in a different ROS topic and subscribed here?
            # We should probably delay here too
            # genome.fitness = ...

if __name__ == '__main__':
    neatNode = NEATNode()
    
