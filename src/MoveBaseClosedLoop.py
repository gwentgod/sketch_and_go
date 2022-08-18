#!/usr/bin/env python3

import numpy as np
import rospy as ros
from geometry_msgs.msg import Twist
from sketch_and_go.msg import Int64Array
from rospy.numpy_msg import numpy_msg

SPEED = 0.15
RATE = 0.3


class MoveBaseClient:
    def reset(self):
        self.current_index = 0
        self.milestones = np.empty(0)
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.pub.publish(self.cmd)

    def __init__(self):
        self.cmd = Twist()
        self.yaw = 0

        ros.init_node('move_base_client')

        self.sub = ros.Subscriber('/map_milestones', numpy_msg(Int64Array), self.milestones_cb, queue_size=2)
        self.pub = ros.Publisher('/mobile_base/commands/velocity', Twist, queue_size=2)

        self.rate = ros.Rate(10)

        self.reset()

        while not ros.is_shutdown():
            if self.current_index == self.milestones.shape[0] - 1:
                self.reset()
            if self.milestones.size != 0:
                diff = self.milestones[self.current_index + 1] - self.milestones[self.current_index]
                dist = np.linalg.norm(diff)
                yaw = np.arctan2(diff[1], diff[0])

                angdiff = yaw - self.yaw
                angdiff %= 2*np.pi
                if 0.1 < angdiff < 6.27:
                    self.yaw = yaw
                    if angdiff < np.pi:
                        self.cmd.angular.z = np.pi / 8
                    else:
                        self.cmd.angular.z = - np.pi / 8
                        angdiff = 2*np.pi - angdiff
                    self.cmd.linear.x = 0
                    for _ in range(int((angdiff * 8 / np.pi * 10) + 8)):
                        self.pub.publish(self.cmd)
                        ros.loginfo(self.cmd)
                        self.rate.sleep()

                self.cmd.linear.x = SPEED
                self.cmd.angular.z = 0
                for _ in range(int((dist * RATE) + 10)):
                    self.pub.publish(self.cmd)
                    ros.loginfo(self.cmd)
                    self.rate.sleep()
                self.current_index += 1
            else:
                self.rate.sleep()

    def milestones_cb(self, msg):
        self.reset()
        self.milestones = np.copy(msg.data)
        self.milestones = self.milestones.reshape((self.milestones.size // 2, 2))
        self.milestones[:, 1] = 512 - self.milestones[:, 1]


if __name__ == '__main__':
    MoveBaseClient()
