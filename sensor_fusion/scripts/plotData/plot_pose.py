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

data_pose = np.loadtxt("../data/pose_position.txt", delimiter=' ', dtype=np.float)
colors = ['red', 'blue', 'green']

t_pose = data_pose[:,0]
x_pose = data_pose[:,1]
y_pose = data_pose[:,2]

t = t_pose
x = x_pose

dx = np.zeros(x.shape,np.float)
dx[0:-1] = np.diff(x)/np.diff(t)
dx[-1] = (x[-1] - x[-2])/(t[-1] - t[-2])

ddx = np.zeros(dx.shape,np.float)
ddx[0:-1] = np.diff(dx)/np.diff(t)
ddx[-1] = (dx[-1] - dx[-2])/(t[-1] - t[-2])

t = t_pose
y = y_pose

dy = np.zeros(y.shape,np.float)
dy[0:-1] = np.diff(y)/np.diff(t)
dy[-1] = (y[-1] - y[-2])/(t[-1] - t[-2])

ddy = np.zeros(dy.shape,np.float)
ddy[0:-1] = np.diff(dy)/np.diff(t)
ddy[-1] = (dy[-1] - dy[-2])/(t[-1] - t[-2])

f_plot(t_pose, x_pose, colors=colors, linewidth=2.)
f_plot(t_pose, y_pose,colors=colors, linewidth=2.)
f_plot(t_pose, dx, colors=colors, linewidth=2.)
f_plot(t_pose, dy,colors=colors, linewidth=2.)
f_plot(t_pose, ddx, colors=colors, linewidth=2.)
f_plot(t_pose, ddy,colors=colors, linewidth=2.)
f_plot(x_pose, y_pose, colors=colors, linewidth=2.)
plt.show()

