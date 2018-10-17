#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
#sudo chmod 666 /dev/ttyACM0

def forward(vel_msg, velocity_publisher, speed, distance):
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # Setting the current calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    vel_msg.linear.x = abs(speed)

    # Loop to move the turtle in an specified distance
    while (current_distance < distance):
        # Publish the velocity
        velocity_publisher.publish(vel_msg)
        # Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        # Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    
    # After the loop, stops the robot.
    vel_msg.linear.x = 0

    velocity_publisher.publish(vel_msg)

    return True

def turn(vel_msg, velocity_publisher):
    angular_speed = 1
    relative_angle = math.pi

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # # Checking if our movement is CW or CCW
    # if clockwise:
    #     vel_msg.angular.z = -abs(angular_speed)
    # else:
    #     vel_msg.angular.z = abs(angular_speed)
    
    vel_msg.angular.z = -abs(angular_speed)
    
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.spin()

    return True   


#def callback(data):
#    rospy.loginfo(data)

def movement(input_values, velocity_publisher, odom_subscriber):

    # Values for the TwisOdomet Odometry messages 
    vel_msg = Twist()
    odom_msg = Odometry()
    
    while abs(odom_msg.pose.pose.position.x) < input_values.square_size_x  or abs(odom_msg.pose.pose.position.y) < input_values.square_size_y:   
        #do something
        True
        



class InputValues:
    # Easier access to the provides values
    def __init__(speed, square_size_x, square_size_y, angular_speed, relative_angle):
        self.speed = speed
        self.square_size_x = square_size_x
        self.square_size_y = square_size_y
        self.angular_speed = angular_speed
        self.relative_angle = relative_angle

def robotInit():
    # Init a node to subscribe to the topic odom.
    # Position and orientation is recieved from there 
    rospy.init_node('odom_subscriber', anonymous=True)
    odom_subscriber = rospy.Subscriber('odom', Odometry, callback=None)

    # Init a node to publish control commands to the robot
    rospy.init_node('move_publisher', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)


    PI = math.pi


    # Receiveing the user's input
    print("Let's move your robot")
    speed = input("Input your speed(m/s):")
    print("The robot will start in the middle of the area")
    square_size_x = input("Input the length of the area")
    square_size_y = input("Input the width of the area")
    
    # How the robot will turn on the edges
    anglespeed = input("Input your speed (degrees/sec):")
    angle = input("Type your distance (degrees):")
    clockwise = input("Clockwise?: ") #True or false

    # Converting from angles toOdomet radians
    angular_speed = anglespeed*Odomet2*PI/360
    relative_angle = angle*2*PI/360

    input_values = InputValues(speed, square_size_x, square_size_y, angular_speed, relative_angle)
    
    movement(input_values, odom_subscriber, velocity_publisher)

    return True    

if __name__ == '__main__':
    try:
        # Initialize the nodes and values 
        robotInit()
    except rospy.ROSInterruptException: pass

        
