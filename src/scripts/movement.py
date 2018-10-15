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
    
    # Force the robot to stop
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

def move():
    rospy.init_node('move', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move your robot")
    speed = input("Input your speed:")
    distance = input("Type your distance:")
    #isForward = input("Foward?: ")#True or False
    
    # #Checking if the movement is forwaawards
    # if(isForward):
    #     vel_msg.linear.x = abs(speed)
    # else:
    #     vel_msg.linear.x = -abs(speed)

    forward(vel_msg, velocity_publisher, speed, distance)
    turn(vel_msg, velocity_publisher)
    forward(vel_msg, velocity_publisher, speed, distance)
    turn(vel_msg, velocity_publisher)

    return True

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass

        