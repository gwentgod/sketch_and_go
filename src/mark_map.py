#!/home/steve/miniconda3/envs/ros/bin/python

from glob import glob
from tkinter.messagebox import NO
import cv2 as cv
import numpy as np
import rospy as ros
import rospkg
from rospy.numpy_msg import numpy_msg
from mouse_commander.msg import Float64Array


rospack = rospkg.RosPack()
pack_path = rospack.get_path('mouse_commander')
map_img = cv.imread(pack_path+'/house.pgm')
map_img = cv.resize(map_img, (map_img.shape[0]*2, map_img.shape[1]*2))
map_copy = np.copy(map_img)
milestones = None
updated = True

def show_proj(msg):
    global milestones, updated
    pts = msg.data
    pts = pts.reshape((pts.size//2, 2))
    updated = False
    milestones = ((pts + np.array([10, -9])) * np.array([40, -40])).astype(int)

if __name__ == '__main__':
    ros.init_node('map_marker')
    sub = ros.Subscriber('map_milestones', numpy_msg(Float64Array), show_proj, queue_size=2)
    rate = ros.Rate(24)
    while cv.waitKey(1):
        if not updated:
            map_copy = np.copy(map_img)
            color = (0, 150, 255)
            map_copy = np.copy(map_img)
            for i in range(milestones.shape[0]):
                cv.circle(map_copy, tuple(milestones[i]), 5, color, -1)
                if i > 0:
                    cv.line(map_copy, tuple(milestones[i - 1]), tuple(milestones[i]), color, 1)
            updated = True
        cv.imshow('Map', map_copy)
