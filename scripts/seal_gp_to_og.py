#!/usr/bin/env python3
# license removed for brevity
__author__ = 'Ehsan Latif'
__version__ = '1.0'
#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

class OccupancyGridNode:
    def __init__(self):
        rospy.init_node('occupancy_grid_node', anonymous=True)
        self.map_pub = rospy.Publisher('/occupancy_grid_map', OccupancyGrid, queue_size=10)
        self.position_sub = rospy.Subscriber('/dgorl/position', PoseStamped, self.position_callback, queue_size=1)
        self.gp_sub = rospy.Subscriber('/gp_value', Float64, self.gp_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        self.current_position = None
        self.current_gp_value = None
        self.current_odom = None
        self.current_scan = None
        self.rate = rospy.Rate(10)

    def position_callback(self, msg):
        self.current_position = msg.pose.position

    def gp_callback(self, msg):
        self.current_gp_value = msg.data

    def odom_callback(self, msg):
        self.current_odom = msg.pose.pose.position

    def scan_callback(self, msg):
        self.current_scan = msg.ranges

    def create_occupancy_grid(self):
        if self.current_position is None or self.current_gp_value is None or self.current_scan is None:
            return

        grid_size = 100
        resolution = 0.1
        grid_map = np.zeros((grid_size, grid_size), dtype=np.int8)

        for i, r in enumerate(self.current_scan):
            angle = i * np.pi / len(self.current_scan)
            x = self.current_position.x + r * np.cos(angle)
            y = self.current_position.y + r * np.sin(angle)
            grid_x = int(x / resolution)
            grid_y = int(y / resolution)

            if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                grid_map[grid_x, grid_y] = int(self.current_gp_value * 100)

        return grid_map

    def publish_occupancy_grid(self, grid_map):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "odom"
        map_msg.info.resolution = 0.1
        map_msg.info.width = grid_map.shape[1]
        map_msg.info.height = grid_map.shape[0]
        map_msg.data = grid_map.flatten().tolist()

        self.map_pub.publish(map_msg)

    def run(self):
        while not rospy.is_shutdown():
            grid_map = self.create_occupancy_grid()
            if grid_map is not None:
                self.publish_occupancy_grid(grid_map)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        occupancy_grid_node = OccupancyGridNode()
        occupancy_grid_node.run()
    except rospy.ROSInterruptException:
        pass
