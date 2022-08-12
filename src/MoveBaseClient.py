import numpy as np
from scipy.spatial.transform import Rotation

import rospy as ros
import actionlib
from move_base_msgs.msg import *

from Projector import Projector


class MoveBaseClient:
    def __init__(self):
        self.projector = Projector()

        self.is_active = False
        self.stuck = False
        self.pose = None

        self.reset()

        self.goal_msg = MoveBaseGoal()
        self.goal_msg.target_pose.header.frame_id = 'map'

        self.action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        ros.loginfo('Connecting to action server')
        self.action_client.wait_for_server()

        self.timer = ros.Timer(ros.Duration(2.5), self.check_stuck)

    def reset(self):
        if self.is_active:
            self.action_client.cancel_goal()
            self.is_active = False

        self.current_index = 0
        self.stuck = False
        self.last_pose = None
        ros.loginfo('Reseted')

    def set_milestones(self, milestones):
        self.milestones = self.projector.project(milestones)
        if not self.is_active:
            self.is_active = True
            self.send_goal()
        ros.loginfo('Got new milestones')

    def feedback_callback(self, feedback):
        self.pose = np.array([feedback.base_position.pose.position.x, feedback.base_position.pose.position.x])

    def active_callback(self):
        pass

    def done_callback(self, status, result):
        if self.current_index < self.milestones.shape[0]-1 and status != 2:
            self.current_index += 1
            self.send_goal()
        else:
            self.is_active = False
            ros.loginfo("Finished")

    def send_goal(self):
        if self.is_active:
            current_milestone = self.milestones[self.current_index]
            self.goal_msg.target_pose.pose.position.x = current_milestone[0]
            self.goal_msg.target_pose.pose.position.y = current_milestone[1]
            
            if self.current_index < self.milestones.shape[0]-1:
                diff = self.milestones[self.current_index+1] - current_milestone
            else:
                diff = np.array([0, 1])
            r = Rotation.from_euler('z', np.arctan2(*diff[::-1]))
            quat = r.as_quat()
            self.goal_msg.target_pose.pose.orientation.x = quat[0]
            self.goal_msg.target_pose.pose.orientation.y = quat[1]
            self.goal_msg.target_pose.pose.orientation.z = quat[2]
            self.goal_msg.target_pose.pose.orientation.w = quat[3]
            self.action_client.send_goal(self.goal_msg,
                                         feedback_cb=self.feedback_callback,
                                         active_cb=self.active_callback,
                                         done_cb=self.done_callback)

    def check_stuck(self, event):
        if self.is_active:
            if self.last_pose is not None:
                displacement = np.linalg.norm(self.pose - self.last_pose)
                if displacement < 0.004:
                    ros.logwarn("I'm stuck! Please plan a new path!")
                    self.stuck = True
                    return
            self.last_pose = self.pose
