#!/home/steve/miniconda3/envs/ros/bin/python

import cv2 as cv
import numpy as np

import rospy as ros
from rospy.numpy_msg import numpy_msg
from mouse_commander.msg import Int64Array


class Sketchpad:
    def reset(self):
        self.pad = np.ones((512, 512, 3)) * 255
        self.path = np.empty((0, 2), dtype=np.uint)
        self.milestones = np.empty((0, 2))

    def __init__(self):
        self.mouse_down = False
        ros.init_node('sketchpad')
        self.publisher = ros.Publisher('map_milestones', numpy_msg(Int64Array), queue_size=2)
        self.reset()

        cv.namedWindow('Sketchpad')
        cv.setMouseCallback('Sketchpad', self.mouse_callback)

        while cv.waitKey(1) & 0xff != 27:
            cv.imshow('Sketchpad', self.pad)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.reset()
            self.mouse_down = True

        elif event == cv.EVENT_MOUSEMOVE and self.mouse_down:
            if self.milestones.size == 0:
                last_point = np.uint([[x, y]])
                self.path = np.concatenate((self.path, last_point))
                if self.path.shape[0] >= 2:
                    cv.line(self.pad, tuple(self.path[-2]), tuple(self.path[-1]), color=(204, 102, 0), thickness=3)

        elif event == cv.EVENT_LBUTTONUP:
            self.mouse_down = False
            if self.milestones.size == 0:
                self.get_milestones()

    def draw_path(self):
        if self.path.shape[0] >= 2:
            for i in range(1, self.path.shape[0]):
                cv.line(self.pad, tuple(self.path[i - 1]), tuple(self.path[i]), color=(0, 255, 255), thickness=2)

    def get_milestones(self, space=20):
        if self.path.size == 0:
            return

        if self.path.size < space:
            self.milestones = np.array([self.path[0], self.path[-1]])
        else:
            self.milestones = self.path[::space]
            self.milestones[-1] = self.path[-1]
        self.publisher.publish(self.milestones.flatten())

    def mark_milestones(self):
        for i in range(self.milestones.shape[0]):
            color = (0, 255, 255)  # yellow
            cv.circle(self.pad, tuple(self.milestones[i]), 5, color, -1)
            if i > 0:
                cv.line(self.pad, tuple(self.milestones[i - 1]), tuple(self.milestones[i]), color, 2)


if __name__ == '__main__':
    Sketchpad()
