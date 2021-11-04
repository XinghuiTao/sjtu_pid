#! /usr/bin/env python

import cv2
import numpy as np
import time
import math


class Plot:
    def __init__(self, w=640, h=480, y_limit=[-60, 60], ros_rate=10):
        self.w = w
        self.h = h
        self.y_limit = y_limit
        self.step = h/(self.y_limit[1]-y_limit[0])
        self.ros_rate = ros_rate

        self.plot = np.zeros((self.h, self.w, 3), np.uint8)
        self.plot[:] = 225, 225, 225

        self.x = 0
        self.y = 0
        self.x_logs = [x for x in range(0, 100)]
        self.y_logs = []
        self.time = 0             

    def update(self, current_y):
        self.drawBackground()

        self.y = int(np.interp(current_y, self.y_limit, [0, self.h]))
        self.y_logs.append(self.y)
        if len(self.y_logs) == 100: self.y_logs.pop(0)

        for i in range(0, len(self.y_logs)):
            if i < 2:
                pass
            else:
                cv2.line(self.plot, (int((self.x_logs[i - 1] * (self.w // 100))) - (self.w // 10), self.y_logs[i - 1]),
                                    (int((self.x_logs[i] * (self.w // 100)) - (self.w // 10)), self.y_logs[i]), 
                            (255, 0, 255), 2)

        return self.plot

    def drawBackground(self):
        cv2.rectangle(self.plot, (0, 0), (self.w, self.h), (0, 0, 0), cv2.FILLED)
        grid = int(self.step * 10)

        for x in range(0, self.w, grid):
            cv2.line(self.plot, (x, 0), (x, self.h), (50, 50, 50), 1)
        for y in range(0, self.h, grid):
            cv2.line(self.plot, (0, y), (self.w, y), (50, 50, 50), 1)
            scale = f'{int((self.h - y) * ((self.y_limit[1]-self.y_limit[0]) / self.h)) - self.y_limit[1]}'
            cv2.putText(self.plot, scale, (10, y), cv2.FONT_HERSHEY_PLAIN, 1, (150, 150, 150), 1)

        # cv2.line(self.plot, (0, self.h//2), (self.w, self.h//2), (150, 150, 150), 1)
        cv2.line(self.plot, (0, self.h//2-grid), (self.w, self.h//2-grid), (150, 150, 150), 1)
        cv2.line(self.plot, (0, self.h//2+grid), (self.w, self.h//2+grid), (150, 150, 150), 1)