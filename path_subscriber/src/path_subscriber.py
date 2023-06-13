#!/usr/bin/python3

import rospy 
from nav_msgs.msg import Path

def path_callback(msg):
    """
    Callback function for getting path out of the navigation stack
    If the robot is stationary, nothing will be printed
    """
    print("\npath array")
    pose_array = msg.poses

    for pose in pose_array:
        rospy.loginfo("pose: {}".format(pose))

# initialize a ros node which subscribes to the path planner
rospy.init_node('path_subscriber')

rospy.Subscriber('/move_base/NavfnROS/plan', Path, path_callback)

rospy.spin()
