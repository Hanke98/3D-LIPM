from Utils import *


class Link(object):
    def __init__(self, name, end, theta, lb=None, ub=None):
        self.name = name
        self.end = glm.vec4(end)
        self.theta = theta
        self.trans = end[:-1]
        self.pos = end[:-1]
        self.lb = lb if lb is not None else [-90] * 3
        self.ub = ub if ub is not None else [90] * 3
        self.theta_back = self.theta.copy()

    def reset(self):
        self.theta = self.theta_back.copy()


class KineChain(object):
    def __init__(self, root):
        self.links = []
        self.root = root

    def add_link(self, link):
        self.links.append(link)

    def apply_angle(self, ang):
        assert np.shape(ang)[0] == np.shape(self.links)[0]
        for a, l in zip(ang, self.links):
            l.theta += a
            l.theta = np.clip(l.theta, l.lb, l.ub)

    def reset(self):
        [l.reset() for l in self.links]

    def fk(self):
        m = glm.mat4(1)
        m = glm.translate(m, self.root)
        for l in self.links:
            m = rotate(m, l.theta)
            l.pos = to_numpy(m * l.end)
            m = glm.translate(m, l.trans)

    def ik(self, t):
        h = 0.01
        alpha = 0.1
        count = 0
        while not self.ik_step(t, h, alpha):
            # print(count, self.links[-1].pos)
            count += 1
            if count > 1000:
                break

    def ik_step(self, t, h=0.01, alpha=0.1):
        jac = np.zeros((3, len(self.links * 3)))
        self.fk()
        e = self.links[-1].pos
        for i, l in enumerate(self.links):
            for j in range(len(l.theta)):
                l.theta[j] += h
                self.fk()
                e_p = self.links[-1].pos
                d = (e_p - e) / h
                l.theta[j] -= h
                jac[0, i * 3 + j] = d[0]
                jac[1, i * 3 + j] = d[1]
                jac[2, i * 3 + j] = d[2]
        self.fk()
        j_plus = np.dot(jac.T, np.linalg.inv(np.dot(jac, jac.T)))
        de = t - e
        if np.dot(de, de) < 0.0001:
            return True
        dt = np.dot(j_plus, de)
        dt = np.reshape(dt * alpha, (len(self.links), 3))
        self.apply_angle(dt)

    def get_data(self):
        self.fk()
        c = [self.root]
        for l in self.links:
            c = np.vstack((c, l.pos))
        return c.T
