import numpy as np
from scipy.spatial.transform import Rotation

import rospy
from rospy.numpy_msg import numpy_msg
from sketch_and_go.msg import Float64Array

k = np.array([[641.1105893484068, 0.0,               540.5],
              [0.0,               641.1105893484068, 360.5],
              [0.0,               0.0,               1.0]])

r = Rotation.from_euler('x', -0.7).as_matrix()
r = r @ Rotation.from_euler('yzx', [90, 270, 0], degrees=True).as_matrix()
t = np.array([[5, -3, -2]]).T
t = r @ t
rt = np.concatenate((r, t), axis=1)

p = k @ rt
p = p[:, (0,1,3)]
invp = np.linalg.inv(p)

class Projector:
    def __init__(self):
        self.publisher = rospy.Publisher('map_milestones', numpy_msg(Float64Array), queue_size=2)

    def project(self, imgpts):
        pts = np.concatenate((imgpts, np.ones((imgpts.shape[0], 1))), axis=1)
        pts[:,1] = 720 - pts[:,1]
        pts = invp @ pts.T
        pts /= pts[2]
        pts = pts[:2].T

        self.publisher.publish(pts.flatten())

        return pts

