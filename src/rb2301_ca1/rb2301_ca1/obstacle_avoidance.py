from turtle import left

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  
minimum_distance = 0.22 #0.26

side_min = 0.286 #0.326


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message

        #represents whether a dead end on the left side has been encountered
        self.flag = 0

        #node attributes for recentering
        self.prev_time = self.get_clock().now()
        self.vy = 0
        self.y_coord = 0

        self.timer = self.create_timer(0.1, self.timer_callback)  # Runs at 10Hz. Can be changed.

            


    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        self.last_scan = np.array(msg.ranges)[::20] # Slices the 721 scan array to return only 36 scans. Feel free to edit
        self.front = np.concatenate((self.last_scan[:5],self.last_scan[-4:]))
        self.front_left = self.last_scan[:9]
        self.front_right = self.last_scan[-9:]
        self.left = self.last_scan[8:13]
        self.right = self.last_scan[23:28]


    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        
        ######################## MODIFY CODE HERE ########################

        #if front unobstructed, move forward
        if np.all(self.front >= minimum_distance): 
            vy = 0 #initiate variable for y-direction velocity

            #nested block for recentering. use front_left+left and front_right+right to check if recentering towards either direction is clear

            if self.y_coord >= 0.05: #use 0.05m as the tolerable band of deviation from original y-coordinate
                #if nothing on right and front-right, then move diagonally right to recenter IF robot is left of original y-center
                if np.all(self.right >= side_min) and np.all(self.front_right >=minimum_distance):
                    vy = -0.2
            elif self.y_coord<=-0.05:
                if np.all(self.left >= side_min) and np.all(self.front_left >=minimum_distance) :
                    vy = 0.2
            self.move_2D(0.3,vy,0) #robot either moves diagonally left or diagonally right
            self.calculate_y(self.vy,vy) #function definition below
        else: #if front is obstructed
            if self.flag == 0: #robot by default moves left horizontally to avoid obstacles
                #if left is/becomes obstructed before finding an opening in front, means theres a deadend on left, so change flag and go right
                if np.any(self.left <= side_min): 
                    self.flag = 1
                    self.move_2D(0,-0.4,0)
                    self.calculate_y(self.vy, -0.4)
                else:
                    self.move_2D(0,0.4,0)
                    self.calculate_y(self.vy, 0.4)

            else:
                if np.any(self.right<=side_min):
                    self.flag = 0
                    self.move_2D(0,0.4,0)
                    self.calculate_y(self.vy, 0.4)
                else:
                    self.move_2D(0,-0.4,0)
                    self.calculate_y(self.vy, -0.4)
    ######################## MODIFY CODE HERE ########################
    #helper function for tracking y-direction displacement for recentering
    def calculate_y(self, prev_vy, current_vy):
        self.current = self.get_clock().now()
        dt = self.current - self.prev_time
        self.y_coord += (dt.nanoseconds/1e9) * prev_vy #tracks y-coordinate based on integrating prev commanded velocity
        self.prev_time = self.current
        self.vy = current_vy






        



def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()