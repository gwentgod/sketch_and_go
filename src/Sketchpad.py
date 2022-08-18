#!/usr/bin/env python3

import cv2 as cv
import numpy as np

import rospy as ros
import ros_numpy
from sensor_msgs.msg import Image

from MoveBaseClient import MoveBaseClient


class Sketchpad:
    def reset_path(self):
        self.path = np.empty((0, 2), dtype=np.uint)
        self.milestones = np.empty((0, 2))

    def __init__(self):
        self.mouse_down = False
        self.selected = None

        self.reset_path()

        ros.init_node("img_sub")
        self.imgmsg = None
        self.imgsub = ros.Subscriber("/rrbot/camera1/image_raw", Image, self.cam_callback, queue_size=1)

        self.move_base_client = MoveBaseClient()

        cv.namedWindow('Cam')
        cv.setMouseCallback('Cam', self.mouse_callback)

        rate = ros.Rate(24)

        while cv.waitKey(1):
            self.get_cam()
            cv.imshow('Cam', self.img)

    def cam_callback(self, msg):
        self.imgmsg = msg

    def get_cam(self):
        if self.imgmsg:
            # self.img = self.bridge.imgmsg_to_cv2(self.imgmsg, "bgr8")
            self.img = ros_numpy.numpify(self.imgmsg)
            if self.milestones.size > 0:
                self.mark_milestones()
            else:
                self.draw_path()
        else:
            ros.logwarn("no message received")
            self.img = np.zeros((1080, 720, 3))

    def draw_path(self):
        if self.path.shape[0] >= 2:
            for i in range(1, self.path.shape[0]):
                cv.line(self.img, tuple(self.path[i-1]), tuple(self.path[i]), color=(0, 255, 255), thickness=2)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.mouse_down = True
            if self.milestones.size > 0:
                dist = np.linalg.norm(self.milestones - np.array([x, y]), axis=1)
                self.selected = dist.argmin()
                if dist[self.selected] > 20 or self.selected <= self.move_base_client.current_index:
                    self.selected = None

        elif event == cv.EVENT_MOUSEMOVE and self.mouse_down:
            if self.milestones.size == 0:
                last_point = np.uint([[x, y]])
                self.path = np.concatenate((self.path, last_point))
            elif self.selected is not None:
                self.milestones[self.selected] = np.array([x, y])

        elif event == cv.EVENT_LBUTTONUP:
            self.mouse_down = False
            self.selected = None
            if self.milestones.size == 0:
                self.get_milestones()
            self.move_base_client.set_milestones(self.milestones)

        elif event == cv.EVENT_RBUTTONDOWN:
            self.reset_path()
            self.move_base_client.reset()

    def get_milestones(self, space=15):
        if self.path.size == 0:
            return

        if self.path.size < space:
            self.milestones = np.array([self.path[0], self.path[-1]])
        else:
            self.milestones = self.path[::space]
            self.milestones[-1] = self.path[-1]

    def mark_milestones(self):
        for i in range(self.milestones.shape[0]):
            if i < self.move_base_client.current_index:
                color = (255, 200, 10) # blue
            elif i == self.move_base_client.current_index:
                if self.move_base_client.stuck:
                    color = (100, 100, 255) # pink
                else:
                    color = (50, 255, 50) # green
            else:
                color = (0, 255, 255) # yellow
            cv.circle(self.img, tuple(self.milestones[i]), 5, color, -1)
            if i > 0:
                cv.line(self.img, tuple(self.milestones[i - 1]), tuple(self.milestones[i]), color, 2)


if __name__ == '__main__':
    Sketchpad()
