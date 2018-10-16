#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
#sudo chmod 666 /dev/ttyACM0



def callback(data):
    #print("Callback")
    #print(data)
    rate = rospy.Rate(1) # 10hz 
    #print(data.pose.pose)
    print(data)
    rate.sleep()

def listener():
    rospy.init_node('collision_subscriber', anonymous=True)
    #rospy.Subscriber('odom', Odometry, callback)
    rospy.Subscriber('sensor_status', Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass