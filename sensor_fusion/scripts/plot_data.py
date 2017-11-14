#!/usr/bin/env python

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

def f_plot(*args, **kwargs):
    xlist = []
    ylist = []
    for i, arg in enumerate(args):
        if (i % 2 == 0):
            xlist.append(arg)
        else:
            ylist.append(arg)

    colors = kwargs.pop('colors', 'k')
    linewidth = kwargs.pop('linewidth', 1.)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    i = 0
    for x, y, color in zip(xlist, ylist, colors):
        i += 1
        ax.plot(x, y, color=color, linewidth=linewidth, label=str(i))

    ax.grid(True)
    ax.legend()

data_imu = np.loadtxt("IMU.txt", delimiter=' ', dtype=np.float)
data_lidar_odom = np.loadtxt("robot_yaw.txt", delimiter=' ', dtype=np.float)
colors = ['red', 'blue']

x_imu = data_imu[:,0]
y_imu = data_imu[:,1]

x_lidar_odom = data_lidar_odom[:,0]
y_lidar_odom = data_lidar_odom[:,1]

f_plot(x_imu, y_imu,x_lidar_odom, y_lidar_odom, colors=colors, linewidth=2.)
plt.show()