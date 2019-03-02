import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import glm
from Link import Link

fig = plt.figure()
ax = p3.Axes3D(fig)
x_axis = glm.vec3([1, 0, 0])
y_axis = glm.vec3([0, 1, 0])
z_axis = glm.vec3([0, 0, 1])


def to_numpy(m):
    if isinstance(m, glm.tvec4):
        a = [d for d in m]
    else:
        a = [[v[0], v[1], v[2], v[3]] for v in m]
    return np.array(a)


def rotate(m, theta):
    m = glm.rotate(m, glm.radians(theta[0]), x_axis)
    m = glm.rotate(m, glm.radians(theta[1]), y_axis)
    m = glm.rotate(m, glm.radians(theta[2]), z_axis)
    return m


def fk(root, a, b, c):
    m = glm.mat4(1)
    m1 = glm.mat4(1)
    m2 = glm.mat4(1)

    m = glm.translate(m, root)
    m = rotate(m, a.theta)
    ar = to_numpy(m * a.end)

    m1 = glm.translate(m1, a.trans)
    m1 = rotate(m1, b.theta)
    br = to_numpy(m * m1 * b.end)

    m2 = glm.translate(m2, b.trans)
    m2 = rotate(m2, c.theta)
    cr = to_numpy(m * m1 * m2 * c.end)

    xxa = [xyz for xyz in zip(root, ar)]
    ax.plot(xxa[0], xxa[1], xxa[2])
    xxb = [xyz for xyz in zip(ar, br)]
    ax.plot(xxb[0], xxb[1], xxb[2])
    xxc = [xyz for xyz in zip(br, cr)]
    ax.plot(xxc[0], xxc[1], xxc[2])
    ax.scatter(0, 0, 0)
    plt.show()


if __name__ == '__main__':
    theta1 = [0, 0, 90]
    theta2 = [0, 0, 45]
    theta3 = [0, 0, 45]
    ra = [0, -2, 0, 1]
    rb = [0, -1.5, 0, 1]
    rc = [0, -1, 0, 1]

    a = Link(ra, theta1)
    b = Link(rb, theta2)
    c = Link(rc, theta3)
    o = [1, -1, 0]

    fk(o, a, b, c)
