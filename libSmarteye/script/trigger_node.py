#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from smarteye.srv import *

def add_two_ints_client():
    rospy.wait_for_service('/hv1000/get_pointcloud')
    try:
        get_point_cloud = rospy.ServiceProxy('/hv1000/get_pointcloud', GetPointCloud)
        resp = get_point_cloud()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node("fake_video_stream")
    rate = rospy.Rate(2) # ROS Rate at 5Hz
    while not rospy.is_shutdown():
        rospy.loginfo("taking picture...")
        add_two_ints_client()
        rate.sleep()