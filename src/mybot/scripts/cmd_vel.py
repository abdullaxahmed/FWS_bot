#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_robot():
    rospy.init_node('move_robot_node')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(2)
    move = Twist()
    move.linear.x = 0.5  
    move.angular.z = 0.5  

    start_time = time.time()  # Record the start time

    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time > 5:  # Stop after 5 seconds
            rospy.loginfo("Stopping the robot after 5 seconds")
            break
        pub.publish(move)
        rate.sleep()

    # Publish zero velocity to stop the robot
    stop_move = Twist()
    pub.publish(stop_move)
    rospy.loginfo("Robot has stopped")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
