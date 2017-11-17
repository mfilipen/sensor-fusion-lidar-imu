#!/usr/bin/env python

import rospy
from visual import *
import math

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rad2degrees = 180.0/math.pi
yaw_offset = 0 #used to align animation upon key press

magnetometer = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/dataFromROStoTxt/data/magnetometer.txt', 'w')
magnetometer_cavariance = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/dataFromROStoTxt/data/magnetometer_covariance.txt', 'w')

gyro = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/dataFromROStoTxt/data/gyro.txt', 'w')
gyro_cavariance = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/dataFromROStoTxt/data/gyro_covariance.txt', 'w')

accelerometer = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/dataFromROStoTxt/data/accelerometer.txt', 'w')
accelerometer_cavariance = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/dataFromROStoTxt/data/accelerometer_covariance.txt', 'w')

def processIMU_message(imuMsg):
    global yaw_offset
    global f

    roll=0
    pitch=0
    yaw=0

    quaternion = (
      imuMsg.orientation.x,
      imuMsg.orientation.y,
      imuMsg.orientation.z,
      imuMsg.orientation.w)

    (sec, nsec) = (imuMsg.header.stamp.secs, imuMsg.header.stamp.nsecs)
    time = 1. / 1000000000 * nsec + sec

    (roll,pitch,yaw) = euler_from_quaternion(quaternion)

    magnetometer.write("{:.9f} {:.9f} {:.9f} {:.9f}\n".format(time, yaw, roll, pitch))
    magnetometer_cavariance.write("{}\n".format(imuMsg.orientation_covariance))

    accelerometer.write("{:.9f} {:.9f} {:.9f} {:.9f}\n".format(time, imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z))
    accelerometer_cavariance.write("{}\n".format(imuMsg.angular_velocity_covariance))

    gyro.write("{:.9f} {:.9f} {:.9f} {:.9f}\n".format(time, imuMsg.angular_velocity.x,imuMsg.angular_velocity.y, imuMsg.angular_velocity.z))
    gyro_cavariance.write("{}\n".format(imuMsg.linear_acceleration_covariance))


rospy.init_node("imuMsgToTxt")
sub = rospy.Subscriber('imu', Imu, processIMU_message)
rospy.spin()
