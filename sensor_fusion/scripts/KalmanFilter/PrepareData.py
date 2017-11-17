# matrix routine was taken from Udacity course.


from math import *
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


yaw_mag_f = np.loadtxt("../data/magnetometer.txt", delimiter=' ', dtype=np.float)
yaw_lidar_f = np.loadtxt("../data/pose_orientation.txt", delimiter=' ', dtype=np.float)
wYaw_gyro_f = np.loadtxt("../data/gyro.txt", delimiter=' ', dtype=np.float)

yaw_mag_write = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/data/yaw_mag_prepared.txt', 'w')
Yaw_gyro_write = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/data/yaw_gyro_prepared.txt', 'w')
wYaw_gyro_write = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/data/wYaw_gyro_prepared.txt', 'w')
yaw_lidar_write = open('/home/maksim/sensor_fusion_ws/src/sensor_fusion/scripts/data/yaw_lidar_prepared.txt', 'w')

colors = ['red', 'blue', 'green', 'yellow']

t_gyro = wYaw_gyro_f[:,0]
wz_gyro = wYaw_gyro_f[:,3]

iwz = np.zeros(wz_gyro .shape,np.float)
for i in range(len(wz_gyro )-1):
    i+=1
    iwz[i]=(wz_gyro [i-1]+wz_gyro [i])*(t_gyro[i]-t_gyro[i-1])/2

yaw_gyro = np.zeros(iwz.shape,np.float)
for i in range(len(iwz)):
    for j in range(i):
        yaw_gyro[i]+=iwz[j]

t_mag = yaw_mag_f[:,0]
yaw_mag = yaw_mag_f[:,1]

for i in range(len(yaw_mag)):
    if (t_mag[i]<1510685634+100 and t_mag[i]>1510685634+54):
        if (yaw_mag[i] > -1):
            yaw_mag[i]-=2*3.14159265

t_lidar = yaw_lidar_f[:,0]
yaw_lidar = yaw_lidar_f[:,1]

for i in range(len(t_lidar)):
    if (t_lidar[i]<1510685634+100 and t_lidar[i]>1510685634+54):
        if (yaw_lidar[i] > -1):
            yaw_lidar[i]-=2*3.14159265



f_plot(t_mag, yaw_mag,t_gyro, yaw_gyro,t_lidar, yaw_lidar, colors=colors, linewidth=2.)
plt.show()

for i in range(len(t_lidar)):
    yaw_lidar_write.write("{:.9f} {:.9f}\n".format(t_lidar[i],yaw_lidar[i]))

for i in range(len(t_mag)):
    if (i%6!=0 and i%126!=125):
        yaw_mag_write.write("{:.9f} {:.9f}\n".format(t_mag[i],yaw_mag[i]))
        Yaw_gyro_write.write("{:.9f} {:.9f}\n".format(t_gyro[i],yaw_gyro[i]))
        wYaw_gyro_write.write("{:.9f} {:.9f}\n".format(t_gyro[i], wz_gyro[i]))




