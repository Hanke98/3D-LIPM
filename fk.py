import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

from Link import *

fig = plt.figure()
ax = p3.Axes3D(fig)


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
    return ar, br, cr

    # xxa = [xyz for xyz in zip(root, ar)]
    # ax.plot(xxa[0], xxa[1], xxa[2])
    # xxb = [xyz for xyz in zip(ar, br)]
    # ax.plot(xxb[0], xxb[1], xxb[2])
    # xxc = [xyz for xyz in zip(br, cr)]
    # ax.plot(xxc[0], xxc[1], xxc[2])
    # ax.scatter(0, 0, 0)
    # plt.show()


def ik(root, l1, l2, l3, target):
    _, _, r = fk(root, l1, l2, l3)
    d = np.sqrt(np.square(r - target))


if __name__ == '__main__':
    theta1 = [0., 0., 45.]
    theta2 = [0., 0., 45.]
    theta3 = [0., 0., 45.]
    ra = [0., -1., 0., 1.]
    rb = [0, -1, 0, 1]
    rc = [0, -1, 0, 1]

    a = Link(ra, theta1)
    b = Link(rb, theta2)
    c = Link(rc, theta3)
    o = [0, 0, 0]
    t = [3, 0, 0]

    # fk(o, a, b, c)
    # ik(o, a, b, c, t)
    chain = KineChain(o)
    chain.add_link(a)
    chain.add_link(b)
    chain.add_link(c)
    chain.ik(t)
    # chain.fk()
    # ar = chain.chain[0].pos
    # br = chain.chain[1].pos
    # cr = chain.chain[2].pos
    #
    # xxa = [xyz for xyz in zip(o, ar)]
    # ax.plot(xxa[0], xxa[1], xxa[2])
    # xxb = [xyz for xyz in zip(ar, br)]
    # ax.plot(xxb[0], xxb[1], xxb[2])
    # xxc = [xyz for xyz in zip(br, cr)]
    # ax.plot(xxc[0], xxc[1], xxc[2])
    # ax.scatter(0, 0, 0)
    # plt.show()
