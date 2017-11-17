# matrix routine was taken from Udacity course.


from math import *
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


class matrix:
    # implements basic operations of a matrix class

    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def first(self):
        return self.value[0][0]

    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise (ValueError, "Invalid size of matrix")
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise (ValueError, "Invalid size of matrix")
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise (ValueError, "Matrices must be of equal dimensions to add")
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise (ValueError, "Matrices must be of equal dimensions to subtract")
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise (ValueError, "Matrices must be m*n and n*p to multiply")
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions

    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i]) ** 2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise (ValueError, "Matrix not positive-definite")
                res.value[i][i] = sqrt(d)
            for j in range(i + 1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S) / res.value[i][i]
        return res

    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k] * res.value[j][k] for k in range(j + 1, self.dimx)])
            res.value[j][j] = 1.0 / tjj ** 2 - S / tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum(
                    [self.value[i][k] * res.value[k][j] for k in range(i + 1, self.dimx)]) / self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)


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


def g(x, u):
    # here can be any nonlinear function
    return x + u


def Jg():
    #  It is first row of Jacobian for our function g(x,u)
    return matrix([[1.]])


def h(x):
    # here can be any nonlinear function
    x_value = x.first()
    return matrix([[x_value], [x_value], [x_value]])


def Jh():
    # It is first row of Jacobian for our function h(x)
    return matrix([[1.], [1.], [1.]])


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

# f_plot(t_mag, yaw_mag, t_gyro, yaw_gyro,t_lidar, yaw_lidar, colors=colors, linewidth=2.)
# plt.show()

# I take data about angular speed from sensor in prediction for simplicity.
# the same data can be calcuted from forward kinematic.
########################################

print("### Extended Kalman filter ###")

H = matrix([[1.],
            [1.],
            [1.]])

R = matrix([[0.01, 0., 0.],
            [0., 0.02, 0.],
            [0., 0., 0.04]])

I = matrix([[1.]])

yaw = np.zeros(t_lidar.shape, np.float)

P_bel = matrix([[0]])

Q = matrix([[0.1]])

for i in range(len(t_lidar) - 1):
    print("{} \n".format(i))
    # prediction

    x = g(matrix([[yaw[i]]]), matrix([[(wYaw_gyro[i] + wYaw_gyro[i + 1]) * 0.5 * (t_gyro[i + 1] - t_gyro[i])]]))
    P = Jg() * P_bel * Jg().transpose() + Q

    # measurement update
    H = Jh()
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()

    Z = matrix([[yaw_lidar[i]],
                [yaw_gyro[i]],
                [yaw_mag[i]]])
    y = Z - h(x)

    yaw[i + 1] = (x + (K * y)).first()
    P_bel = (I - (K * H)) * P

f_plot(t_mag, yaw_mag, t_gyro, yaw_gyro, t_lidar, yaw_lidar, t_lidar, yaw, colors=colors, linewidth=2.)
plt.show()
