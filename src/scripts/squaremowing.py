#!/usr/bin/env python

import math
import signal
import sys
import tf
import random

# ROS imports
import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from am_driver.msg import WheelEncoder
#sudo chmod 666 /dev/ttyACM0

def signal_handler(sig, frame):
        print('Exit')
        sys.exit(0)

class HRQ(object):
    
    def __init__(self, speed, square_size_x, square_size_y, angular_speed, relative_angle):
        self.speed = speed
        self.square_size_x = square_size_x
        self.square_size_y = square_size_y
        self.angular_speed = angular_speed
        self.relative_angle = relative_angle
        self.x, self.y = 0, 0

        # Init a node to subscribe to the topic odom.
        # Position and orientation is recieved from there 
        rospy.init_node('movement', anonymous=True)
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, callback=self.update)
        
        # Subscribe to the wheel_encoder topic
        self.wheel_encoder_subscriber = rospy.Subscriber('wheel_encoder', WheelEncoder, callback = self.update_encoder)

        # Init a node to publish control commands to the robot
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Values for the Twist and Odometry messages 
        self.vel_msg = Twist()
        self.odom_msg = Odometry()
    
    def update(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        
        # Orientation of the robot in quaternion coordinates
        self.qx = data.pose.pose.orientation.x
        self.qy = data.pose.pose.orientation.y
        self.qz = data.pose.pose.orientation.z
        self.qw = data.pose.pose.orientation.w

    
    def update_encoder(self, data):
        # Wheel encoder data
        # Use to check if the robot has stopped
        self.rwheel = data.rwheel
        self.lwheel = data.lwheel


    def is_within_square(self):
        # print("CHECKING IF I AM WITHIN SQUARE")
        # print("X: %s, Y: %s" % (str(self.x), str(self.y)))
        # print("ABS X: %s, ABS Y: %s" % (str(abs(self.x)), str(abs(self.y))))
        # print("SQSIZE_X: %s, SQSIZE_Y: %s" % (str(self.square_size_x/2), str(self.square_size_y/2)))
        # print()
        # rospy.loginfo(self.x)

        if (abs(self.x) <= (self.square_size_x/2) and abs(self.y) <= (self.square_size_y/2)):
            return True
        
        return False

    def move_forward(self):
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

        # Setting the current calculus
        self.vel_msg.linear.x = abs(speed)

        # Loop to move the turtle in an specified distance
        while (self.is_within_square()):
            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
        
        self.stop()

        return True

    def turn(self):
        # We wont use linear components
        self.vel_msg.linear.x=0
        self.vel_msg.linear.y=0
        self.vel_msg.linear.z=0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0

    
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        variable_angle = random.uniform(-math.radians(30), math.radians(30))

        # Checking if our movement is CW or CCW
        self.clockwise = self.turning_direction()
        if self.clockwise:
            self.vel_msg.angular.z = -abs(self.angular_speed)
        else:
            self.vel_msg.angular.z = abs(self.angular_speed)


        while(current_angle < self.relative_angle + variable_angle):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.angular_speed*(t1-t0)

        self.stop()
        
        # Setting the current calculus
        current_distance = 0
        self.vel_msg.linear.x = abs(speed)

        # Not in use
        # if (abs(self.x) > square_size_x/2):
        #     print("--- out of boundarie in x ---")
        #     err_dist = math.sqrt(math.pow(square_size_x/2, 2) + math.pow(square_size_y/2, 2)) - 
        # elif (abs(self.y) > square_size_y/2):
        #     print("--- out of boundarie in y ---")
        #     err_dist = math.sqrt(math.pow(square_size_x/2 - abs(self.x), 2) + math.pow(square_size_y/2 - abs(self.y), 2)) 
        #     square_size_y/2 - abs(self.y)


        # Loop to move the robot for a specified distance.
        # To move back into the square
        while (current_distance < 0.5):
            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            # Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            # Calculates distancePoseStamped
            current_distance= speed*(t1-t0)

        return True   

    def stop(self):
        # Stops the robot.

        # Sets the velocity to zero
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

        # Waits until the wheels have stopped
        # Without the robot will start next movement before finishing the last movement
        while (self.rwheel != 0 and self.lwheel != 0):
            self.velocity_publisher.publish(self.vel_msg)

        return True


    def turning_direction(self):
        # Finds the angle needed to turn back to the square

        # Quaternion coordinates
        quaternion = (
        self.qx,
        self.qy,
        self.qz,
        self.qw)

        # Transform the quaternion coord. to euler coordinates
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # The third euler angle. Yaw angle in roll, pitch, yaw angles
        # The orientation is in 2D so the last angle is our only concern. 
        # The orientation is in respect to the starting pose
        ori_angle = euler[2]

        # The robots angle from the starting pose
        offset_angle = math.atan2(self.y , self.x)  # .atan2(y,x) takes the signs into account

        print("ori_angle: %s, offset_angle: %s" % (str(ori_angle), str(offset_angle)))

        # Determines to turn clockwise or counter clockwise depending on position and orientation.
        if(abs(ori_angle) <= abs(offset_angle)):
            return 0  # Counter Clockwise
        else:
            return 1 # Clockwise
        


    def main(self):
        
        while(True):
            self.move_forward()
            self.turn()


if __name__ == '__main__':
    try:
        # Enable ctrl+c
        signal.signal(signal.SIGINT, signal_handler)

        # Receiveing the user's input
        print("Let's move your robot")
        speed = 0.5 #input("Input your speed(m/s):")
        print("The robot will start in the middle of the area")
        square_size_x = 1.5 # input("Input the length of the area: ")
        square_size_y = 1.5 # input("Input the width of the area: ")

        # How the robot will turn on the edges
        anglespeed = 50 # input("Input your speed (degrees/sec): ")
        angle = 180 # input("Type your distance (degrees): ")
        # clockwise = 1 # input("Clockwise?: ") #True or false

        PI = math.pi
  
        # Converting from angles toOdomet radians
        angular_speed = anglespeed*2*PI/360
        relative_angle = angle*2*PI/360

        hrq = HRQ(speed, square_size_x, square_size_y, angular_speed, relative_angle)
        hrq.main()

    except rospy.ROSInterruptException: pass
