# I base my code on this books
# https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python


import matplotlib.pyplot as plt
import numpy as np
from numpy.random import uniform
from scipy.stats import norm
from numpy.random import seed


def f_plot(*args, **kwargs):
    xlist = []
    ylist = []
    for i, arg in enumerate(args):
        if (i % 2 == 0):
            xlist.append(arg)
        else:
            ylist.append(narmalization(arg))

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


def narmalization(z):
    for i in range(len(z)):
        if (z[i] < -3.14159265):
            z[i] += 2 * 3.14159265
    return z


def create_uniform_particles(yaw_range, N):
    particles = np.empty((N, 1))
    particles[:, 0] = uniform(yaw_range[0], yaw_range[1], size=N)
    return particles


def predict(particles, wz_yaw, std):
    # update heading
    particles[:, 0] += wz_yaw + (np.random.randn(len(particles)) * std)

    # for normalization
    for i in range(len(particles)):
        if (particles[i, 0] > np.pi):
            particles[i, 0] -= 2 * np.pi


def update(particles, weights, z, R, sensor_data):
    weights.fill(1.)
    for i, landmark in enumerate(sensor_data):
        distance = particles[:, 0] - landmark
        weights *= norm(distance, R).pdf(z[i])

    weights += 1.e-300  # avoid round-off to zero
    weights /= sum(weights)  # normalize


def estimate(particles, weights):
    yaw = particles[:, 0]
    mean = np.average(yaw, weights=weights, axis=0)
    var = np.average((yaw - mean) ** 2, weights=weights, axis=0)
    return mean, var


def simple_resample(particles, weights):
    N = len(particles)
    cumulative_sum = np.cumsum(weights)
    cumulative_sum[-1] = 1.  # avoid round-off error
    indexes = np.searchsorted(cumulative_sum, np.random.random(N))

    # resample according to indexes
    particles[:] = particles[indexes]
    weights.fill(1.0 / N)


def neff(weights):
    return 1. / np.sum(np.square(weights))


def systematic_resample(weights):
    N = len(weights)

    # make N subdivisions, and choose positions with a consistent random offset
    positions = (np.random.random() + np.arange(N)) / N

    indexes = np.zeros(N, 'i')
    cumulative_sum = np.cumsum(weights)
    i, j = 0, 0
    while i < N:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes


def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights.fill(1.0 / len(weights))


def ParticalFilter(N, sensor_std_err=.05,
                   gyro=None, lidar=None,
                   mag=None, wYaw=None, t=None):


    particles = create_uniform_particles(((-1) * np.pi, np.pi), N)
    weights = np.zeros(N)

    xs = []
    robot_yaw = np.array([0.])
    for i in range(len(lidar)):
        print("{} \n".format(i))
        sensor_data = np.array([gyro[i], lidar[i], mag[i]])

        zs = sensor_data - robot_yaw + sensor_std_err

        # predict(particles, wz_yaw=wYaw[i]*(t[2]-t[1]), std=.1)
        predict(particles, wz_yaw=0.00, std=.1)

        # incorporate measurements
        update(particles, weights, z=zs, R=sensor_std_err, sensor_data=sensor_data)

        # resample if too few effective particles
        if neff(weights) < N / 2:
            indexes = systematic_resample(weights)
            resample_from_index(particles, weights, indexes)

        mu, var = estimate(particles, weights)
        xs.append(mu * 0.5)

    xs = np.array(xs)
    return xs


print("### Read data ###")

yaw_mag_f = np.loadtxt("../data/yaw_mag_prepared.txt", delimiter=' ', dtype=np.float)
yaw_lidar_f = np.loadtxt("../data/yaw_lidar_prepared.txt", delimiter=' ', dtype=np.float)
Yaw_gyro_f = np.loadtxt("../data/yaw_gyro_prepared.txt", delimiter=' ', dtype=np.float)
wYaw_gyro_f = np.loadtxt("../data/wYaw_gyro_prepared.txt", delimiter=' ', dtype=np.float)
colors = ['red', 'blue', 'green', 'yellow']

t_gyro = Yaw_gyro_f[:, 0]
yaw_gyro = Yaw_gyro_f[:, 1]
wYaw_gyro = wYaw_gyro_f[:, 1]

t_mag = yaw_mag_f[:, 0]
yaw_mag = yaw_mag_f[:, 1]

t_lidar = yaw_lidar_f[:, 0]
yaw_lidar = yaw_lidar_f[:, 1]

########################################

print("### Partical filter ###")

seed(1)
yaw = ParticalFilter(N=1000, gyro=yaw_gyro, mag=yaw_mag, lidar=yaw_lidar, wYaw=wYaw_gyro, t=t_gyro)

f_plot(t_mag, yaw_mag, t_gyro, yaw_gyro, t_lidar, yaw_lidar, t_lidar, yaw, colors=colors, linewidth=2.)
plt.show()
