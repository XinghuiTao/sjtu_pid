#! /usr/bin/env python

class PID:
    def __init__(self, ros_rate=10):
        self.ros_rate = ros_rate
        self.prev_error = 0
        self.I = 0

    def update(self, pid_params, current_val, target_val):
        error = current_val - target_val
        
        interval = 1/self.ros_rate
        
        P = error
        self.I = self.I + error*interval
        D = (error-self.prev_error)/interval
        PID = pid_params[0]*P + pid_params[1]*self.I + pid_params[2]*D

        self.prev_error = error

        return float(PID)
