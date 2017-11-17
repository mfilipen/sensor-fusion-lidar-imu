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

data_acc = np.loadtxt("../data/accelerometer.txt", delimiter=' ', dtype=np.float)
colors = ['red', 'blue', 'green']

t_acc = data_acc[:,0]
x_acc = data_acc[:,1]
y_acc = data_acc[:,2]

ix = np.zeros(x_acc.shape,np.float)
for i in range(len(x_acc)-1):
    i+=1
    ix[i]=(x_acc[i-1]+x_acc[i])*(t_acc[i]-t_acc[i-1])/2

vx = np.zeros(ix.shape,np.float)
for i in range(len(ix)):
    for j in range(i):
        vx[i]+=ix[j]

iix = np.zeros(vx.shape,np.float)
for i in range(len(vx)-1):
    i+=1
    iix[i]=(vx[i-1]+vx[i])*(t_acc[i]-t_acc[i-1])/2

x = np.zeros(iix.shape,np.float)
for i in range(len(iix)):
    for j in range(i):
        x[i]+=iix[j]

iy = np.zeros(y_acc.shape,np.float)
for i in range(len(t_acc)-1):
    i+=1
    iy[i]=(y_acc[i-1]+y_acc[i])*(t_acc[i]-t_acc[i-1])/2

vy = np.zeros(iy.shape,np.float)
for i in range(len(iy)):
    for j in range(i):
        vy[i]+=iy[j]

iiy = np.zeros(vy.shape,np.float)
for i in range(len(t_acc)-1):
    i+=1
    iiy[i]=(vy[i-1]+vy[i])*(t_acc[i]-t_acc[i-1])/2

y = np.zeros(iiy.shape,np.float)
for i in range(len(iiy)):
    for j in range(i):
        y[i]+=iiy[j]

f_plot(t_acc,x_acc, colors=colors, linewidth=2.)
f_plot(t_acc , y_acc, colors=colors, linewidth=2.)
f_plot(t_acc, vx, colors=colors, linewidth=2.)
f_plot(t_acc , vy, colors=colors, linewidth=2.)
f_plot(t_acc , y, colors=colors, linewidth=2.)
f_plot(t_acc , x, colors=colors, linewidth=2.)
f_plot(x , y, colors=colors, linewidth=2.)



plt.show()

