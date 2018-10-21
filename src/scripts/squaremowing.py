#!/usr/bin/env python

import math
import signal
import sys
import tf
import random
import pygame, pygame.locals

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

class GraphicInterface:
    
    def __init__(self):
        # constants
        self.JET = (54, 54, 53)
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.width, self.height = 640, 512
        self.hqr = None
    
    def run(self):
        # pygame window
        pygame.init()

        self.font = pygame.font.SysFont('Ubuntu', 24)
        self.font_14 = pygame.font.SysFont('Ubuntu', 14)
        self.position_x_title = self.font.render('Position_X:', True, self.WHITE)
        self.position_y_title = self.font.render('Position_Y:', True, self.WHITE)
        self.orientation_angle_title = self.font.render('Orientation_Angle:', True, self.WHITE)

        self.screen = pygame.display.set_mode((self.width, self.height), 0, 32)
        pygame.display.set_caption('ROS Group 15')
        return
    
    def set_key_repeat(self):
        pygame.key.set_repeat(500,30)
        return
    
    def quit(self):
        pygame.quit()
        sys.exit()

    def controller(self):
        while(True):
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_s:
                        print('[INFO] Moving started..')
                        return self.hqr.move()
                    elif event.key == pygame.K_q:
                        print('[INFO] Quit')
                        self.hqr.stop()
                        return self.quit()
                elif event.type == pygame.locals.QUIT:
                    self.hqr.stop()
                    return self.quit()
            
            self.screen.fill(self.JET)
            self.draw_values()
            pygame.display.flip()
    
    def draw_values(self):
        self.screen.fill(self.JET)
        self.screen.blit(self.position_x_title, (10, 10))
        self.screen.blit(self.font.render(str(hqr.x), True, self.WHITE), (130, 10))
        self.screen.blit(self.position_y_title, (10, 50))
        self.screen.blit(self.font.render(str(hqr.y), True, self.WHITE), (130, 50))
        self.screen.blit(self.orientation_angle_title, (10,90))
        self.screen.blit(self.font.render(str(hqr.ori_angle*(180/math.pi))+' degrees', True, self.WHITE), (220, 90))
        pygame.display.flip()
        return
    
class HQR(object):
    def __init__(self, speed, square_size_x, square_size_y, angular_speed, relative_angle):
        self.speed = speed
        self.square_size_x = square_size_x
        self.square_size_y = square_size_y
        self.angular_speed = angular_speed
        self.relative_angle = relative_angle
        self.x, self.y = 0, 0
        self.rwheel, self.lwheel = 0, 0
        self.ori_angle = 0
        self.graphic_interface = None

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
        # Updates the pose data continuously 
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        
        # Orientation of the robot in quaternion coordinates
        self.qx = data.pose.pose.orientation.x
        self.qy = data.pose.pose.orientation.y
        self.qz = data.pose.pose.orientation.z
        self.qw = data.pose.pose.orientation.w
    
    def update_encoder(self, data):
        # Wheel encoder data
        # Use to see when the robot has stopped
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
        # Setting the current calculus
        self.vel_msg.linear.x = abs(speed)
        self.graphic_interface.draw_values()

        # Loop to move while it is within the boundaries
        while (self.is_within_square()):
            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            self.graphic_interface.draw_values()
        
        self.stop()

        return True

    def turn(self):

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

        # Make a random turn in the interval [150, 210] as the
        # initial angle is been set 180
        while(current_angle < self.relative_angle + variable_angle):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.angular_speed*(t1-t0)
            self.graphic_interface.draw_values()

        self.stop()
        
        # Not in use
        # if (abs(self.x) > square_size_x/2):
        #     print("--- out of boundarie in x ---")
        #     err_dist = math.sqrt(math.pow(square_size_x/2, 2) + math.pow(square_size_y/2, 2)) - 
        # elif (abs(self.y) > square_size_y/2):
        #     print("--- out of boundarie in y ---")
        #     err_dist = math.sqrt(math.pow(square_size_x/2 - abs(self.x), 2) + math.pow(square_size_y/2 - abs(self.y), 2)) 
        #     square_size_y/2 - abs(self.y)

        # Setting the current calculus
        current_distance = 0
        self.vel_msg.linear.x = abs(speed)
        t0 = rospy.Time.now().to_sec()
        
        # Loop to move the robot for a specified distance.
        # To move back into the square
        while (current_distance < 0.5):
            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            # Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            # Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
            self.graphic_interface.draw_values()

        return True   

    def stop(self):
        # Stops the robot.

        # Sets all velocities to zero
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

        self.velocity_publisher.publish(self.vel_msg)

        # Waits until the wheels have stopped
        # Without the robot will start next movement before finishing the last movement
        while (self.rwheel != 0 and self.lwheel != 0):
            self.velocity_publisher.publish(self.vel_msg)

        return True

    def turning_direction(self):
        # Finds the shorter turning direction to turn back into the circle

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
        self.ori_angle = euler[2]

        self.graphic_interface.draw_values()

        # The robots angle from the starting pose
        offset_angle = math.atan2(self.y , self.x)  # .atan2(y,x) takes the signs into account

        # print("self.ori_angle: %s, offset_angle: %s" % (str(self.ori_angle), str(offset_angle)))

        # Determines to turn clockwise or counter clockwise depending on position and orientation.
        if(abs(self.ori_angle) <= abs(offset_angle)):
            return 0  # Counter Clockwise
        else:
            return 1 # Clockwise

    def move(self):
        while(True):
            self.move_forward()
            self.turn()
            for event in pygame.event.get():
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_p:
                        print('[INFO] Pause')
                        self.stop()
                        return self.graphic_interface.controller()
                    elif event.key == pygame.K_q:
                        print('[INFO] Quit')
                        self.stop()
                        return self.graphic_interface.quit()
                break
        return
    
    def main(self):
        if self.graphic_interface:
            self.graphic_interface.run()
            self.graphic_interface.controller()
        else:
            self.move()
        
        return

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
  
        # Converting from angles to radians
        angular_speed = anglespeed*2*PI/360
        relative_angle = angle*2*PI/360

        hqr = HQR(speed, square_size_x, square_size_y, angular_speed, relative_angle)
        graphic_interface = GraphicInterface()

        # SET TO None IF NO GRAPHIC INTERFACE IS DESIRED
        # The robot will start moving until the program is closed.
        hqr.graphic_interface = graphic_interface
        
        # Give the graphic interface the robot object so it can control it
        # todo: make this an api with abstraction and interfaces.
        graphic_interface.hqr = hqr
        
        hqr.main()

    except rospy.ROSInterruptException: pass
