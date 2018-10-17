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



class HRQ(object):
    
    def __init__(self, speed, square_size_x, square_size_y, angular_speed, relative_angle, clockwise):
        self.speed = speed
        self.square_size_x = square_size_x
        self.square_size_y = square_size_y
        self.angular_speed = angular_speed
        self.relative_angle = relative_angle
        self.clockwise = clockwise
        self.x, self.y = 0, 0

        # Init a node to subscribe to the topic odom.
        # Position and orientation is recieved from there 
        rospy.init_node('movement', anonymous=True)
        odom_subscriber = rospy.Subscriber('odom', Odometry, callback=self.update)

        # Init a node to publish control commands to the robot
        velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.odom_subscriber = odom_subscriber
        self.velocity_publisher = velocity_publisher
        
        # Values for the Twist and Odometry messages 
        self.vel_msg = Twist()
        self.odom_msg = Odometry()
    
    def update(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    def is_within_square(self):
        print("CHECKING IF I AM WITHIN SQUARE")
        print("X: %s, Y: %s" % (str(self.x), str(self.y)))
        print("ABS X: %s, ABS Y: %s" % (str(abs(self.x)), str(abs(self.y))))
        print("SQSIZE_X: %s, SQSIZE_Y: %s" % (str(self.square_size_x/2), str(self.square_size_y/2)))
        print()
        rospy.loginfo(self.x)

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
        print("AM I?")
        k = 0
        while (self.is_within_square()):
            print("YES I AM")
            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            k += 1
        
        self.stop()

        return True

    def turn(self, clockwise=True):
        # We wont use linear components
        self.vel_msg.linear.x=0
        self.vel_msg.linear.y=0
        self.vel_msg.linear.z=0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            self.vel_msg.angular.z = -abs(self.angular_speed)
        else:
            self.vel_msg.angular.z = abs(self.angular_speed)
    
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.angular_speed*(t1-t0)
        
        # Setting the current calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        self.vel_msg.linear.x = abs(speed)

        # Loop to move the turtle in an specified distance
        while (current_distance < 0.2):
            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            # Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            # Calculates distancePoseStamped
            current_distance= speed*(t1-t0)

        self.stop()

        return True   

    def stop(self):
        # stops the robot.
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

        return True

    def main(self):

        while(True):
            self.move_forward()
            self.turn()


if __name__ == '__main__':
    try:
        # Receiveing the user's input
        print("Let's move your robot")
        speed = 0.5 #input("Input your speed(m/s):")
        print("The robot will start in the middle of the area")
        square_size_x = 2 # input("Input the length of the area: ")
        square_size_y = 2 # input("Input the width of the area: ")

        # How the robot will turn on the edges
        anglespeed = 50 # input("Input your speed (degrees/sec): ")
        angle = 150 # input("Type your distance (degrees): ")
        clockwise = 1 # input("Clockwise?: ") #True or false

        PI = math.pi
  
        # Converting from angles toOdomet radians
        angular_speed = anglespeed*2*PI/360
        relative_angle = angle*2*PI/360

        hrq = HRQ(speed, square_size_x, square_size_y, angular_speed, relative_angle, clockwise)
        hrq.main()

    except rospy.ROSInterruptException: pass


