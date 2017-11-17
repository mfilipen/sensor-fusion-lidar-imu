#!/usr/bin/env python

import rospy
from visual import *
import math

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

rad2degrees = 180.0/math.pi
yaw_offset = 0 #used to align animation upon key press

pose_position = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/dataFromROStoTxt/data/pose_position.txt', 'w')
pose_orientation = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/dataFromROStoTxt/data/pose_orientation.txt', 'w')
pose_covariance = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/dataFromROStoTxt/data/pose_covariance.txt', 'w')

def processPoseWithCovarianceStamped_message(dataMsg):
    global yaw_offset
    global f

    roll=0
    pitch=0
    yaw=0

    quaternion = (
        dataMsg.pose.pose.orientation.x,
        dataMsg.pose.pose.orientation.y,
        dataMsg.pose.pose.orientation.z,
        dataMsg.pose.pose.orientation.w)

    (roll, pitch, yaw) = euler_from_quaternion(quaternion)

    (sec,nsec)=(dataMsg.header.stamp.secs,dataMsg.header.stamp.nsecs)

    time = 1./1000000000 * nsec + sec

    (roll,pitch,yaw) = euler_from_quaternion(quaternion)

    pose_position.write("{:.9f} {:.9f} {:.9f} {:.9f}\n".format(time, dataMsg.pose.pose.position.x, dataMsg.pose.pose.position.y, dataMsg.pose.pose.position.z))
    pose_orientation.write("{:.9f} {:.9f} {:.9f} {:.9f}\n".format(time, yaw, roll, pitch))
    pose_covariance.write("{}\n".format(dataMsg.pose.covariance))



rospy.init_node("poseupdateMsgToTxt")
sub = rospy.Subscriber('poseupdate', PoseWithCovarianceStamped, processPoseWithCovarianceStamped_message)
rospy.spin()
