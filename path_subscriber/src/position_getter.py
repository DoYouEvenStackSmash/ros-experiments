#!/usr/bin/python3
import rospy
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped

rospy.init_node('robot_position_listener')

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

rospy.sleep(1.0)

while not rospy.is_shutdown():
    try:
        transformed_pose = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    except tf2_ros.TransformException as e:
        rospy.logerr("Transform Failed {}".format(e))
        continue
    position = transformed_pose.transform.translation
    rospy.loginfo("robot position - x:{}, y:{}, z:{}".format(position.x, position.y, position.z))
