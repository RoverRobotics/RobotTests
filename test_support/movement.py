import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MovementManager:
    def __init__(self, max_linear_speed = 0.5, max_angular_speed = 0.628):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

        # create node for communicating with the robot..
        self.tpub = rospy.Publisher('/managed/key', Twist, queue_size=2)
        rospy.init_node('movement_test')

        # create node for listening to the robot
        self.tsub = rospy.Subscriber('/rr_openrover_driver/odom_encoder', Odometry, self.callback)
        self.last_message_received_time = time.time()
        self.accumulated_time = 0

        # meters
        self.xtraveled_total = 0
        self.xtraveled_last_movement = 0
        self.last_x_linear = 0
        # radians 
        self.ztraveled_total = 0 
        self.ztraveled_last_movement = 0
        self.last_z_angular = 0

    def callback(self, data):
        
        # manage time
        current_message_time = time.time()
        
        current_x_linear = data.twist.twist.linear.x
        current_z_angular = data.twist.twist.angular.z

        dt = current_message_time - self.last_message_received_time
        dx = (current_x_linear + self.last_x_linear) * dt / 2
        dz = (current_z_angular + self.last_z_angular) * dt / 2

        self.xtraveled_last_movement += dx
        self.ztraveled_last_movement += dz
        self.accumulated_time += dt

        self.last_message_received_time = current_message_time
        self.last_x_linear = current_x_linear
        self.last_z_angular = current_z_angular

    def set_max_linear_velocity(self, velocity):
        self.max_linear_speed = velocity

    def set_max_angular_velocity(self, velocity):
        self.max_angular_speed = velocity

    def move_straight(self, distance=0):
        self.xtraveled_last_movement = 0
        self.ztraveled_last_movement = 0

        sendmsg = Twist()
        sendmsg.angular.z = 0
        sendmsg.linear.x = self.max_linear_speed if distance >= 0 else -self.max_linear_speed

        while not rospy.is_shutdown() and abs(self.xtraveled_last_movement) < abs(distance):
            self.tpub.publish(sendmsg)
            rospy.sleep(.00001)  

        sendmsg.angular.z = 0
        sendmsg.linear.x = 0
        self.tpub.publish(sendmsg)

    def turn(self, distance=0):
        self.xtraveled_last_movement = 0
        self.ztraveled_last_movement = 0

        sendmsg = Twist()
        sendmsg.angular.z = self.max_angular_speed if distance >= 0 else -self.max_angular_speed
        sendmsg.linear.x = 0

        while not rospy.is_shutdown() and abs(self.ztraveled_last_movement) < abs(distance):
            self.tpub.publish(sendmsg)
            rospy.sleep(.00001)  

        sendmsg.angular.z = 0
        sendmsg.linear.x = 0
        self.tpub.publish(sendmsg)