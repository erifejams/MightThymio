import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry


import sys

##sensor
from sensor_msgs.msg import Range #use to get one range reading of the distance measured 

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node2')

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        self.rightProximity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, '/thymio0/cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, '/thymio0/odom', self.odom_callback, 10)

        ##created a sensor which will call range, which will call the specified function everytime a reading is gotten from the robots sensors
        self.proximity_center_subscriber = self.create_subscription(Range, '/thymio0/proximity/center', self.proximity_center_callback, 10)
        self.proximity_centerRight_subscriber = self.create_subscription(Range, '/thymio0/proximity/right', self.proximity_center_right_callback, 10)
        self.proximity_centerLeft_subscriber = self.create_subscription(Range, '/thymio0/proximity/left', self.proximity_center_left_callback, 10)

        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    ##checking sensor to get the distance
    def proximity_center_callback(self, msg):
        self.centerProximity = msg.range
        return self.centerProximity
    
    def proximity_center_right_callback(self, msg):
        self.rightProximity = msg.range 
        return self.rightProximity
        

    def proximity_center_left_callback(self, msg):
        self.leftProximity = msg.range
        return self.leftProximity


    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
        
    def update_callback(self):
        # robot is moving straight ahead
        cmd_vel = Twist() 
        cmd_vel.linear.x  = 2.0# [m/s]
        cmd_vel.angular.z = 0.0 # [rad/s]

        self.vel_publisher.publish(cmd_vel) 

        if self.rightProximity is None:
            return 

        ##check if wall is detected
        ##reason for this on the visualisation of rqt, if the wall is not detected, it always stays on -1, but 
        # error showing up that ControllerNode' object has no attribute 'rightProximity' don't know why??
        if self.rightProximity != -1.0 or self.leftProximity != -1.0 or self.centerProximity != -1.0:
            self.get_logger().info('wall detected')
            # Set all velocities to zero
            cmd_vel.linear.x  = 0.0# [m/s]
            self.vel_publisher.publish(cmd_vel) #stop robot from moving
           
         
            ##the robot x-axis should be orthogonal to the wall
            if self.rightProximity > self.leftProximity: 
                cmd_vel.linear.x  = 2.0
                cmd_vel.angular.z = 5.0
                self.vel_publisher.publish(cmd_vel)
            elif self.leftProximity > self.rightProximity:
                cmd_vel.linear.x  = 2.0
                cmd_vel.angular.z = -5.0
                self.vel_publisher.publish(cmd_vel)
            else: ###if the wall is center, so it didn't twist left or right
                cmd_vel = Twist()
                self.vel_publisher.publish(cmd_vel)
    
        elif all([self.rightProximity, self.leftProximity, self.centerProximity]): ##==None
            self.get_logger().info('no wall detected')
    



def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
