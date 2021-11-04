#! /usr/bin/env python

class PID:
    def __init__(self, ros_rate=10):
        self.ros_rate = ros_rate
        self.prev_error = 0
        self.I = 0

    def update(self, pid_params, current_val, target_val):
        error = current_val - target_val
        interval = 1/self.ros_rate
        self.prev_error = error

        P = pid_params[0]*error
        self.I = self.I + pid_params[1]*error*interval
        D = pid_params[2]*(error-self.prev_error)/interval
        PID = P + self.I + D

        return float(PID)
