#!/usr/bin/env python3
# license removed for brevity
__author__ = 'Ehsan Latif'
__version__ = '1.0'


#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial import ConvexHull
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Float64

class ConvexHullNode:
    def __init__(self):
        rospy.init_node('convex_hull_node', anonymous=True)
        self.convex_hull_pub = rospy.Publisher('/convex_hull', PolygonStamped, queue_size=10)
        self.gp_sub = rospy.Subscriber('/gp_value', Float64, self.gp_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        self.current_gp_value = None
        self.current_odom = None
        self.current_scan = None
        self.rate = rospy.Rate(10)

    def gp_callback(self, msg):
        self.current_gp_value = msg.data

    def odom_callback(self, msg):
        self.current_odom = msg.pose.pose.position

    def scan_callback(self, msg):
        self.current_scan = msg.ranges

    def create_convex_hull(self):
        if self.current_odom is None or self.current_scan is None:
            return

        points = []
        for i, r in enumerate(self.current_scan):
            angle = i * np.pi / len(self.current_scan)
            x = self.current_odom.x + r * np.cos(angle)
            y = self.current_odom.y + r * np.sin(angle)
            points.append([x, y])

        points = np.array(points)
        hull = ConvexHull(points)
        return points[hull.vertices]

    def publish_convex_hull(self, convex_hull_points):
        polygon_msg = PolygonStamped()
        polygon_msg.header.stamp = rospy.Time.now()
        polygon_msg.header.frame_id = "odom"
        for point in convex_hull_points:
            point32 = Point32()
            point32.x = point[0]
            point32.y = point[1]
            polygon_msg.polygon.points.append(point32)

        self.convex_hull_pub.publish(polygon_msg)

    def run(self):
        while not rospy.is_shutdown():
            convex_hull_points = self.create_convex_hull()
            if convex_hull_points is not None:
                self.publish_convex_hull(convex_hull_points)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        convex_hull_node = ConvexHullNode()
        convex_hull_node.run()
    except rospy.ROSInterruptException:
        pass
