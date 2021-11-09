#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cvlib.object_detection import draw_bbox

from math import degrees, radians, atan
import array
import numpy as np
import statistics
import time

import random
RANDOM_SEED = 42
random.seed(RANDOM_SEED)

from helpers.utils import GazeboConnection
gazebo = GazeboConnection()

from helpers.cvlib import Detection
detection = Detection()
fpv = [320, 480]

from helpers.control import Control
control = Control()
hz = 10

from simple_pid import PID

from deap import base, creator, tools, algorithms
IND_SIZE=3
POPULATION_SIZE = 3
EVALUATION_LENGTH = 10
MAX_GENERATIONS = 50
P_CROSSOVER = 0.9
P_MUTATION = 0.1

# population
creator.create("FitnessMax", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)

toolbox = base.Toolbox()
toolbox.register("pid_float", random.uniform, 0, 1)
toolbox.register("individualCreator", tools.initRepeat, creator.Individual, toolbox.pid_float, n=IND_SIZE)
toolbox.register("populationCreator", tools.initRepeat, list, toolbox.individualCreator)

toolbox.register("select", tools.selTournament, tournsize=2)
toolbox.register("mate", tools.cxOnePoint)
toolbox.register("mutate", tools.mutShuffleIndexes, indpb=1.0/IND_SIZE)


class Yaw(object):
    def __init__(self):
        rospy.init_node('yaw_node', anonymous=True)
        self.rate = rospy.Rate(hz)

        rospy.Subscriber("/drone/front_camera/image_raw",Image,self.cam_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_msg = Twist()

        self.yaw_angle_pid = 0
        self.frame_id = 0
        self.yaw_logs = []
        
        self.generation_id = 0
        self.population = toolbox.populationCreator(n=POPULATION_SIZE)
        self.population_id = 0
        self.fitnessValues = []
        self.offspring = []
        self.maxFitness = 0
        self.meanFitness = 0
        self.maxFitnessValues = []
        self.meanFitnessValues = []

        gazebo.resetSim()
        control.takeoff()
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            if self.frame is None:
                continue
            if self.population_id < POPULATION_SIZE:
                frame = deepcopy(self.frame)
                centroids = detection.detect(frame)               

                if self.frame_id < EVALUATION_LENGTH:
                    if len(centroids)==0:
                        # To-do: fill in gaps
                        self.move_msg.angular.z = 0
                        self.pub_cmd_vel.publish(self.move_msg)
                    else:
                        cent = centroids[0]
                        pid = self.init_population(self.population_id)                    
                        pid_x = pid(cent[0])

                        self.yaw_angle_pid = degrees(atan(pid_x/(fpv[1]-cent[1])))
                        self.move_msg.angular.z = radians(self.yaw_angle_pid)*hz
                        self.pub_cmd_vel.publish(self.move_msg)
                        self.yaw_logs.append(self.yaw_angle_pid)
                    # frame counter
                    self.frame_id = self.frame_id + 1                   
                else:
                    yaw_logs_preprocessing = np.trim_zeros(np.array(self.yaw_logs))
                    std = statistics.stdev(yaw_logs_preprocessing)
                    self.fitnessValues.append(std)
                    # reset
                    self.frame_id = 0
                    self.population_id = self.population_id + 1
                    gazebo.resetSim()
            else:
                    self.maxFitness = max(self.fitnessValues)
                    self.meanFitness = sum(self.fitnessValues) / len(self.population)
                    print("- Generation {}: Max Fitness = {}, Avg Fitness = {}".format(self.generation_id, self.maxFitness, self.meanFitness))
                    
                    
                    self.offspring = toolbox.select(self.population, len(self.population))
                    self.offspring = list(map(toolbox.clone, self.offspring))

                    for child1, child2 in zip(self.offspring[::2], self.offspring[1::2]):
                        if random.random() < P_CROSSOVER:
                            toolbox.mate(child1, child2)
                    
                    for mutant in self.offspring:
                        if random.random() < P_MUTATION:
                            toolbox.mutate(mutant)
                            del mutant.fitness.values

                    self.generation_id = self.generation_id + 1
                    self.population = self.offspring
                    self.population_id = 0
                    self.fitnessValues = []
                    self.offspring = []
                    self.maxFitness = 0
                    self.meanFitness = 0
                    

            self.rate.sleep()
    
    def cam_callback(self, data):
        try:
            cv_img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_img
    
    def init_population(self, population_id):
        pid_params = self.population[population_id]
        pid = PID(pid_params[0], pid_params[1], pid_params[2], setpoint=fpv[0])
        pid.sample_time = 1/hz                       
        return pid
                        
    
    def shutdown(self):
        control.land()
        

def main():
    try:
        Yaw()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main()

# PID
# The proportional term (Kp*proportional_error): helps us to reduce the rise time. 
# The integral term(Ki*integral_error): helps us to reduce any steady-state error.
# The derivative term(Kd*derivative_error): helps us to prevents any overshoot.

# GA
# [Start] Generate random population of n chromosomes (suitable solutions for the problem)
# [Fitness] Evaluate the fitness f(x) of each chromosome x in the population
#  [New population] Create a new population by repeating following steps until the new population is complete
#    [Selection] Select two parent chromosomes from a population according to their fitness (the better fitness, the bigger chance to be selected)
#    [Crossover] With a crossover probability cross over the parents to form a new offspring (children). If no crossover was performed, offspring is an exact copy
#    [Mutation] With a mutation probability mutate new offspring at each locus (position in chromosome).
#    [Accepting] Place new offspring in a new population
# [Replace] Use new generated population for a further run of algorithm
# [Test] If the end condition is satisfied, stop, and return the best solution in current population
# [Loop] Go to step 2