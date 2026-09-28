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
minimum_distance = 0.26

left_min = 0.326


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message

        #subscribe to pose
        # self.sub_pose = self.create_subscription(Odometry, "odom", self.sub_pose_callback, 10) # The subscriber to the pose message.
        # self.last_pose = None # Copied pose message
        # self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.sub_cmd_vel_callback, 10) # The subscriber to the cmd_vel message.
        # self.last_cmd_vel = None 

        #turning counter
        self.turn_left = 0

        self.flag = 0

        self.prev_time = self.get_clock().now()
        self.vy = 0
        self.y_coord = 0


    

        self.timer = self.create_timer(0.025, self.timer_callback)  # Runs at 20Hz. Can be changed.

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
        self.front = np.concatenate((self.last_scan[:4],self.last_scan[-4:]))
        self.front_left = self.last_scan[:9]
        self.front_right = self.last_scan[-9:]
        self.left = self.last_scan[8:13]
        self.right = self.last_scan[23:28]


    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################


        #new draft
        if np.all(self.front >= minimum_distance):
            self.move_2D(0.3,0,0)
            self.calculate_y(self.vy,0)
            #nested block for recentering. use front_left+left and front_right+right
            #can i just check if there is nth on the left or right?
            if self.y_coord >= 0.05:
                if np.all(self.right >= left_min) and np.all(self.front_right >=minimum_distance):
                    self.move_2D(0.3,-0.2,0)
                    self.calculate_y(self.vy,-0.2)
            elif self.y_coord<=-0.05:
                if np.all(self.left >= left_min) and np.all(self.front_left >=minimum_distance) :
                    self.move_2D(0.3,0.2,0)
                    self.calculate_y(self.vy,0.2)


        else:
            if self.flag == 0:
                if np.all(self.front_left>=minimum_distance):
                    self.move_2D(0,0.4,0)
                    self.calculate_y(self.vy, 0.4)

                elif np.all(self.front_right>=minimum_distance):
                    self.move_2D(0,-0.4,0)
                    self.calculate_y(self.vy, -0.4)

                if np.any(self.left <= left_min):
                    self.flag = 1
                    self.move_2D(0,-0.4,0)
                    self.calculate_y(self.vy, -0.4)
                else:
                    self.move_2D(0,0.4,0)
                    self.calculate_y(self.vy, 0.4)

            else:
                if np.all(self.front_right>=minimum_distance):
                    self.move_2D(0,-0.4,0)
                    self.calculate_y(self.vy, -0.4)
                if np.any(self.right<=left_min):
                    self.flag = 0
                    self.move_2D(0,0.4,0)
                    self.calculate_y(self.vy, 0.4)
                else:
                    self.move_2D(0,-0.4,0)
                    self.calculate_y(self.vy, -0.4)

    def calculate_y(self, prev_vy, current_vy):
        self.current = self.get_clock().now()
        dt = self.current - self.prev_time
        self.y_coord += (dt.nanoseconds/1e9) * prev_vy
        self.prev_time = self.current
        self.vy = current_vy






        ######################## MODIFY CODE HERE ########################



def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()