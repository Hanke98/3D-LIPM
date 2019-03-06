from Link import *
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import mpl_toolkits.mplot3d.axes3d as p3
from LIPM import LIPM

fig = plt.figure()
ax = p3.Axes3D(fig)


class Robot:
    def __init__(self, root):
        self.chains = []
        self.root = root
        self.lipm = LIPM()
        self.te, _ = self.lipm.calc_te_tbn()
        self.com_x, self.com_y, self.sf = self.lipm.calc_x()
        self.count = 0
        self.h = self.lipm.h
        self.serial_length = len(self.com_x)
        self.target = []

    def config(self, path):
        data = json.load(open(path, mode='r'))
        for c in data:
            chain = KineChain(self.root)
            for l in c:
                name = l['name']
                pos = l['pos']
                theta = l['theta']
                lb = l['lb']
                ub = l['ub']
                link = Link(name, pos, theta, lb, ub)
                chain.add_link(link)
            self.chains.append(chain)

    def get_data(self):
        return [c.get_data() for c in self.chains]

    def ik(self, target):
        assert len(target) == len(self.chains)
        for c, t in zip(self.chains, target):
            c.root = self.root
            c.ik(t)

    def step(self):
        self.root = [self.com_x[self.count], self.com_y[self.count], self.h]
        stance_pos, _ = self.lipm.get_current_origin()
        self.lipm.update()
        swing_pos = self.sf[self.count]
        self.target = [stance_pos, swing_pos] if stance_pos[1] > 0 else [swing_pos, stance_pos]
        self.count += 1
        self.ik(self.target)
        return self.get_data()

    def reset(self):
        self.count = 0
        self.lipm.reset()
        [c.reset() for c in self.chains]


if __name__ == '__main__':
    root = [0, 0.04, 0.4]
    robot = Robot(root)
    robot.config('config/robot.json')
    ax.set_xlim(0, 1)
    ax.set_ylim(-0.25, 0.25)
    ax.set_zlim(0, 0.5)
    com = ax.plot(robot.com_x, robot.com_y, np.ones_like(robot.com_x) * robot.h)
    left_plt, = ax.plot([0, 0], [0, 0], [0, 0], 'ro-')
    right_plt, = ax.plot([0, 0], [0, 0], [0, 0], 'go-')
    lt_plt, = ax.plot([0, 0], [0, 0], [0, 0], 'b--')
    rt_plt, = ax.plot([0, 0], [0, 0], [0, 0], 'b--')

    lfx = []
    lfy = []
    lfz = []
    rfx = []
    rfy = []
    rfz = []

    def update(newd):
        left_plt.set_data(newd[0][0], newd[0][1])
        left_plt.set_3d_properties(newd[0][-1])
        right_plt.set_data(newd[1][0], newd[1][1])
        right_plt.set_3d_properties(newd[1][-1])
        lfx.append(robot.target[0][0])
        lfy.append(robot.target[0][1])
        lfz.append(robot.target[0][2])
        rfx.append(robot.target[1][0])
        rfy.append(robot.target[1][1])
        rfz.append(robot.target[1][2])

        lt_plt.set_data(lfx, lfy)
        rt_plt.set_data(rfx, rfy)
        lt_plt.set_3d_properties(lfz)
        rt_plt.set_3d_properties(rfz)
        ax.view_init(elev=15, azim=35*(1-robot.count/robot.serial_length))
        return left_plt, right_plt,


    def init():
        robot.reset()
        ax.set_xlim(0, 0.5)
        ax.set_ylim(-0.25, 0.25)
        ax.set_zlim(0, 0.5)
        # ax.view_init(elev=10, azim=10)

        lfx.clear()
        lfy.clear()
        lfz.clear()
        rfx.clear()
        rfy.clear()
        rfz.clear()
        return


    def gen_dot():
        for i in range(robot.serial_length):
            legs = robot.step()
            yield legs


    ani = animation.FuncAnimation(fig, update,
                                  frames=gen_dot,
                                  interval=robot.te,
                                  init_func=init,
                                  save_count=robot.serial_length)
    ani.save('./videos/walk5.mp4', writer='ffmpeg', fps=120)

    plt.show()
