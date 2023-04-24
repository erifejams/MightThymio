import rclpy
from rclpy.node import Node

import math
from math import sqrt, sin, cos, atan2
from geometry_msgs.msg import Twist, Pose



class ControllerNode(Node):


    def __init__(self):
        super().__init__('open_controller')

        # Create a publisher for the topic '/turtle1/cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, '/thymio0/cmd_vel', 10)

        # Create a subscriber to the topic '/turtle1/pose', which will call 
        # self.pose_callback every time a message of type Pose is received
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
    


    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
        
    def update_callback(self):
        """Main update callback: called periodically by the timer to update the controller."""
        return
    
    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)
        


    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt((goal_pose.x - current_pose.x) ** 2 +
                (goal_pose.y - current_pose.y) ** 2)

    def angular_difference(self, goal_theta, current_theta):
        """Compute shortest rotation from orientation current_theta to orientation goal_theta"""
        return atan2(sin(goal_theta - current_theta), cos(goal_theta - current_theta))

    def linear_vel(self, goal_pose, current_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)

    def angular_vel(self, goal_pose, current_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        goal_theta = self.steering_angle(goal_pose, current_pose)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    rclpy.spin(node)


if __name__ == '__main__':
    main()