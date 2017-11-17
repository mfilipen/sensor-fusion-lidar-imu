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

data_acc = np.loadtxt("../data/gyro.txt", delimiter=' ', dtype=np.float)
colors = ['red', 'blue', 'green']

t_acc = data_acc[:,0]
x_acc = data_acc[:,1]
y_acc = data_acc[:,2]
z_acc = data_acc[:,3]


iz = np.zeros(z_acc.shape,np.float)
for i in range(len(z_acc)-1):
    i+=1
    iz[i]=(z_acc[i-1]+z_acc[i])*(t_acc[i]-t_acc[i-1])/2

z = np.zeros(iz.shape,np.float)
for i in range(len(iz)):
    for j in range(i):
        z[i]+=iz[j]


f_plot(t_acc, z_acc , colors=colors, linewidth=2.)
f_plot(t_acc, z , colors=colors, linewidth=2.)

for i in range(len(z)):
    if (z[i]<-3.14159265):
        z[i]+=2*3.14159265

f_plot(t_acc, z , colors=colors, linewidth=2.)

plt.show()

