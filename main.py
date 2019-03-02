import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import mpl_toolkits.mplot3d.axes3d as p3
from LIPM import LIPM

fig = plt.figure()
ax = p3.Axes3D(fig)


def main():
    model = LIPM()
    te, tbn = model.calc_te_tbn()
    x, y = model.calc_x()
    dt = te / model.fra_per_sec
    z = np.ones_like(y) * model.h
    l = ax.plot(x, y, z)
    lipm, = ax.plot([0, 0], [0, 0], [0, model.h], 'ro-')
    color = ['r', 'g']

    def update(newd):
        lipm.set_data([newd[1][0], newd[0][0]], [newd[1][1], newd[0][1]])
        lipm.set_3d_properties([0, model.h])
        lipm.set_color(newd[-1])
        return lipm,

    def init():
        model.reset()
        ax.set_ylim(-0.15, 0.15)
        return l

    def gen_dot():
        for com in zip(x, y):
            model.update()
            cur_origin, cur = model.get_current_origin()
            newdot = [com, cur_origin, color[cur % 2]]
            yield newdot

    ani = animation.FuncAnimation(fig, update,
                                  frames=gen_dot,
                                  interval=dt * model.fra_per_sec,
                                  init_func=init,
                                  save_count=len(x))
    # ani.save('walk2.mp4', writer='ffmpeg', fps=80)
    plt.show()


if __name__ == '__main__':
    main()
