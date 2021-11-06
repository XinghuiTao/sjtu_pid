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

from helpers.cvlib import Detection
detection = Detection()
fpv = [320, 480]

from helpers.control import Control
control = Control()
hz = 10

from simple_pid import PID
pid = PID(0.5, 0.5, 0.4, setpoint=fpv[0])
pid.sample_time = 1/hz

# from deap import base, creator, tools, algorithms
# IND_SIZE=3
# POPULATION_SIZE = 100
# P_CROSSOVER = 0.9
# P_MUTATION = 0.1
# MAX_GENERATIONS = 50

# # population
# creator.create("FitnessMax", base.Fitness, weights=(1.0,))
# creator.create("Individual", list, fitness=creator.FitnessMax)

# toolbox = base.Toolbox()
# toolbox.register("pid_float", random.uniform, 0, 1)
# toolbox.register("individualCreator", tools.initRepeat, creator.Individual, toolbox.pid_float, n=IND_SIZE)
# toolbox.register("populationCreator", tools.initRepeat, list, toolbox.individualCreator)
# population = toolbox.populationCreator(n=POPULATION_SIZE)

# # fitness
# def ISFitness(distance):
#     return sum(distance),
# toolbox.register("evaluate", ISFitness)

# # evolving
# toolbox.register("select", tools.selTournament, tournsize=3)
# toolbox.register("mate", tools.cxOnePoint)
# toolbox.register("mutate", tools.mutShuffleIndexes, indpb=1.0/IND_SIZE)

# # stats
# stats = tools.Statistics(lambda ind: ind.fitness.values)
# stats.register("max", np.max)
# stats.register("mean", np.mean)

# population, logbook = algorithms.eaSimple(population, toolbox, cxpb=P_CROSSOVER, mutpb=P_MUTATION, ngen=MAX_GENERATIONS,
#                                    stats=stats, verbose=True)
# maxFitnessValues, meanFitnessValues = logbook.select("max", "mean")

# print(population)


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

        control.takeoff()
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            if self.frame is not None:
                frame = deepcopy(self.frame)

                centroids = detection.detect(frame)
                if len(centroids)==0:
                    # To-do: fill in gaps
                    self.move_msg.angular.z = 0
                    self.pub_cmd_vel.publish(self.move_msg)
                else:
                    cent = centroids[0]
                    pid_x = pid(cent[0])

                    self.yaw_angle_pid = degrees(atan(pid_x/(fpv[1]-cent[1])))
                    self.move_msg.angular.z = radians(self.yaw_angle_pid)*hz
                    self.pub_cmd_vel.publish(self.move_msg)

                    cv2.circle(frame, (320, cent[1]), 3, [0,0,255], -1, cv2.LINE_AA)
                    cv2.circle(frame, (cent[0], cent[1]), 3, [0,255,0], -1, cv2.LINE_AA)

                cv2.imshow("", frame)
                cv2.waitKey(1)

            self.rate.sleep()
    
    def cam_callback(self,data):
        try:
            cv_img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_img
    
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