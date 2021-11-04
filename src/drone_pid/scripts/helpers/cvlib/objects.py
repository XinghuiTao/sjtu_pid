#! /usr/bin/env python

import cv2
import cvlib as cv
from math import *

class Detection:
    def detect(self, cv_image):
        bboxes, labels, conf = cv.detect_common_objects(cv_image, model='yolov3-tiny', enable_gpu=True)
        indices = cv2.dnn.NMSBoxes(bboxes, conf, score_threshold=0.8, nms_threshold=0.8)

        centroids = []
        for i in indices:
            i = i[0]
            bbox = bboxes[i]
            x1, y1, x2, y2 = bbox
            cent_x = x1 + (x2-x1)/2
            cent_y = y1 + (y2-y1)/2
            centroids.append((int(cent_x), int(cent_y)))
        return centroids