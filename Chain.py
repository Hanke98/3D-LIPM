import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation

from Link import *

fig = plt.figure()
ax = p3.Axes3D(fig)


if __name__ == '__main__':
    theta1 = [0., 0., 45.]
    theta2 = [0., 0., 45.]
    theta3 = [0., 0., 45.]
    ra = [0., -1., 0., 1.]
    rb = [0., -1., 0., 1.]
    rc = [0., -1., 0., 1.]

    a = Link(ra, theta1)
    b = Link(rb, theta2)
    c = Link(rc, theta3)
    o = [0, 0, 0]
    t = [-2, 0, 0.5]

    chain = KineChain(o)
    chain.add_link(a)
    chain.add_link(b)
    chain.add_link(c)
    # chain.ik(t)

    la, = ax.plot([], [], [], 'ro-')
    lb, = ax.plot([], [], [], 'go-')
    lc, = ax.plot([], [], [], 'bo-')


    def init():
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_zlim(-3, 3)
        chain.reset()
        return


    def update(link):
        la.set_data(link[0][:2], link[1][:2])
        lb.set_data(link[0][1:3], link[1][1:3])
        lc.set_data(link[0][2:], link[1][2:])
        la.set_3d_properties(link[2][:2])
        lb.set_3d_properties(link[2][1:3])
        lc.set_3d_properties(link[2][2:])
        return la, lb, lc,


    def gen_dot():
        while not chain.ik_step(t):
            l = [chain.root, chain.chain[0].pos, chain.chain[1].pos, chain.chain[2].pos]
            yield np.transpose(l)


    ani = animation.FuncAnimation(fig, update,
                                  frames=gen_dot,
                                  interval=30,
                                  init_func=init)
    plt.show()
