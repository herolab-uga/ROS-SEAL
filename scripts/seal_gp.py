#!/usr/bin/env python3
# license removed for brevity
__author__ = 'Ehsan Latif'
__version__ = '1.0'


import math
from turtle import circle
#!/usr/bin/env python3

import rospy
import numpy as np
import random
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from collections import deque

class GPTrainerNode:
    def __init__(self):
        rospy.init_node('gp_trainer_node', anonymous=True)
        self.gp_pub = rospy.Publisher('/gp_value', Float64, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        self.kernel = C(1.0, (1e-3, 1e3)) * RBF(1.0, (1e-3, 1e3))
        self.gp = GaussianProcessRegressor(kernel=self.kernel, n_restarts_optimizer=9)

        self.odom_data = deque(maxlen=1000)
        self.scan_data = deque(maxlen=1000)
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        self.odom_data.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def scan_callback(self, msg):
        self.scan_data.append(msg.ranges)

    def train_gp(self):
        X_train = np.array(self.odom_data)
        y_train = np.array([np.mean(scan) for scan in self.scan_data])

        self.gp.fit(X_train, y_train)

    def publish_gp_value(self, position):
        X_test = np.array([position])
        y_pred, _ = self.gp.predict(X_test, return_std=True)

        self.gp_pub.publish(Float64(data=y_pred[0]))

    def run(self):
        while not rospy.is_shutdown():
            if len(self.odom_data) > 0:
                self.train_gp()
                last_position = self.odom_data[-1]
                self.publish_gp_value(last_position)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        gp_trainer_node = GPTrainerNode()
        gp_trainer_node.run()
    except rospy.ROSInterruptException:
        pass
