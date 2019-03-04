import numpy as np
import glm
from Utils import *


class Link:
    def __init__(self, end, theta):
        self.end = glm.vec4(end)
        self.theta = theta
        self.trans = end[:-1]
        self.pos = end[:-1]


class KineChain:
    def __init__(self, root):
        self.chain = []
        self.root = root

    def add_link(self, link):
        self.chain.append(link)

    def apply_angle(self, ang):
        assert np.shape(ang)[0] == np.shape(self.chain)[0]
        for a, l in zip(ang, self.chain):
            l.theta += a

    def fk(self):
        m = glm.mat4(1)
        m = glm.translate(m, self.root)
        for l in self.chain:
            m = rotate(m, l.theta)
            l.pos = to_numpy(m * l.end)
            m = glm.translate(m, l.trans)
        # print(self.chain[-1].pos)

    def ik(self, t):
        h = 0.0001
        alpha = 0.00001
        count = 0
        while True:
            jac = np.zeros((3, len(self.chain * 3)))
            self.fk()
            # print(count, self.chain[-1].pos)
            e = self.chain[-1].pos
            for i, l in enumerate(self.chain):
                for j in range(len(l.theta)):
                    l.theta[j] += h
                    self.fk()
                    e_p = self.chain[-1].pos
                    d = (e_p - e) / h
                    l.theta[j] -= h
                    # self.fk()
                    jac[0, i * 3 + j] = d[0]
                    jac[1, i * 3 + j] = d[1]
                    jac[2, i * 3 + j] = d[2]
            j_plus = np.dot(np.linalg.inv(np.dot(jac.T, jac)), jac.T)
            de = t - e
            if np.dot(de, de) < 0.01:
                break
            dt = np.dot(j_plus, de)
            dt = np.reshape(dt * alpha, (len(self.chain), 3))
            self.apply_angle(dt)
            count += 1
            if count > 100:
                break
