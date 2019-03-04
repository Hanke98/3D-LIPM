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
    x, y, f = model.calc_x()
    dt = te / model.fra_per_sec
    z = np.ones_like(y) * model.h
    l = ax.plot(x, y, z)
    lipm, = ax.plot([0, 0], [0, 0], [0, model.h], 'ro-')
    swing, = ax.plot([0, 0], [0, 0], [0, model.h], 'go-')
    lf_plt, = ax.plot([0, 0], [0, 0], [0, 0], 'b--')
    rf_plt, = ax.plot([0, 0], [0, 0], [0, 0], 'b--')

    lfx = []
    lfy = []
    lfz = []
    rfx = []
    rfy = []
    rfz = []

    def update(newd):
        lipm.set_data([newd[1][0], newd[0][0]], [newd[1][1], newd[0][1]])
        lipm.set_3d_properties([0, model.h])
        swing.set_data([newd[-1][0], newd[0][0]], [newd[-1][1], newd[0][1]])
        swing.set_3d_properties([newd[-1][-1], model.h])

        if newd[-1][1] > 0:
            lfx.append(newd[-1][0])
            lfy.append(newd[-1][1])
            lfz.append(newd[-1][2])
        else:
            rfx.append(newd[-1][0])
            rfy.append(newd[-1][1])
            rfz.append(newd[-1][2])

        lf_plt.set_data(lfx, lfy)
        rf_plt.set_data(rfx, rfy)
        lf_plt.set_3d_properties(lfz)
        rf_plt.set_3d_properties(rfz)

        return lipm, swing, lf_plt, rf_plt,

    def init():
        model.reset()
        lfx.clear()
        lfy.clear()
        lfz.clear()
        rfx.clear()
        rfy.clear()
        rfz.clear()
        ax.set_ylim(-0.15, 0.15)
        return l

    def gen_dot():
        for xx, yy, ff in zip(x, y, f):
            model.update()
            cur_origin, cur = model.get_current_origin()
            newdot = [[xx, yy], cur_origin, ff]
            yield newdot

    ani = animation.FuncAnimation(fig, update,
                                  frames=gen_dot,
                                  interval=100,
                                  init_func=init,
                                  save_count=len(x))
    # ani.save('./videos/walk3.mp4', writer='ffmpeg', fps=80)
    plt.show()


if __name__ == '__main__':
    main()
