import numpy as np


class LIPM:
    def __init__(self):
        self.t = 0
        self.te = 0.1
        self.tbn = 0
        self.h = 0.4
        self.g = 9.81
        self.k = np.sqrt(self.g / self.h)
        self.y0s = [-0.01, 0.01]
        self.x0s = [0, 0.2]
        self.sr = [0.2, -0.1]
        self.epsilon = 0.0001
        self.fra_per_sec = 100
        self.steps = 3
        self.py = [0.05, -0.05]
        self.origin = [0, 0.05]
        self.origin_seq = [self.origin.copy()]
        self.origin_cur = 0
        self.dt = 0.01
        self.max_foot_height = 0.1
        self.legs = [0.25, 0.15, 0.25, 0.15]
        self.theta = [[0, 0, 0] for _ in range(len(self.legs))]

    def calc_te_tbn(self):
        te = 0.1
        i = 0
        y0 = self.y0s[0]
        y0n = self.y0s[1]
        while i < 15:
            tbn = 1 / self.k * np.arcsinh(y0 * np.sinh(self.k * te) / y0n)
            y = y0 * np.cosh(self.k * te) - y0n * np.cosh(self.k * tbn)
            xvte = self.k * y0 * np.sinh(self.k * te)
            dt = (self.sr[1] - y) / 2 / xvte
            te += dt
            if np.abs(dt) < self.epsilon:
                tbn = 1 / self.k * np.arcsinh(y0 * np.sinh(self.k * te) / y0n)
                self.te = te
                self.tbn = tbn
                break
            i += 1
        self.dt = self.te / self.fra_per_sec
        return self.te, self.tbn

    def calc_x(self):
        m = np.eye(4, 4)
        cy = np.ones((4, 1))

        kte = self.k * self.te
        ktbn = self.k * self.tbn
        srx = self.sr[0]
        xv0 = self.k * srx / (np.sinh(kte) - np.cosh(ktbn) * np.tanh(ktbn))

        m[0, 0] = np.cosh(kte)
        m[0, 1] = np.sinh(kte) / self.k
        m[0, 2] = self.te
        m[0, 3] = 1

        m[1, 0] = self.k * np.sinh(kte)
        m[1, 1] = np.cosh(kte)
        m[1, 2] = 1
        m[1, 3] = 0

        m[2, 0] = np.cosh(ktbn)
        m[2, 1] = np.sinh(ktbn) / self.k
        m[2, 2] = self.tbn
        m[2, 3] = 1

        m[3, 0] = self.k * np.sinh(ktbn)
        m[3, 1] = np.cosh(ktbn)
        m[3, 2] = 1
        m[3, 3] = 0

        cy[0, 0] = srx + xv0 / self.k * np.sinh(ktbn)
        cy[1, 0] = xv0 * np.cosh(ktbn)
        cy[2, 0] = xv0 / self.k * np.sinh(ktbn)
        cy[3, 0] = xv0 * np.cosh(ktbn)
        c = np.linalg.solve(m, cy)

        t = 0
        dt = self.te / self.fra_per_sec

        y = []
        x = []
        phase = 0
        count = 0
        foot_swing = []
        while phase < 2 * self.steps:
            kt = self.k * t
            yt = self.y0s[phase % 2] * np.cosh(self.k * t)
            ftx = self.origin[0] - srx + 2 * srx * self.qt(t)
            fty = -self.origin[1]
            ftz = self.lt(t)
            foot_swing.append([ftx, fty, ftz])
            y.append(yt + self.origin[1])

            a = [np.cosh(kt), np.sinh(kt) / self.k, t, 1]
            x.append(np.dot(a, c)[0] + self.origin[0])

            t += dt
            count += 1
            if t > self.te:
                t = self.tbn
                self.origin[0] += srx
                self.origin[1] *= -1
                self.origin_seq[-1].append(count)
                self.origin_seq.append(self.origin.copy())
                phase += 1

        return x, y, foot_swing

    def update(self):
        self.t += self.dt
        if self.t > self.te:
            self.t = self.tbn
            self.origin_cur += 1

    def get_current_origin(self):
        return self.origin_seq[self.origin_cur], self.origin_cur

    def reset(self):
        self.t = 0
        self.origin_cur = 0

    def qt(self, t):
        return 0.5 * (1 - np.cos(np.pi * (t - self.tbn) / (self.te - self.tbn)))

    def lt(self, t):
        return self.max_foot_height * 0.5 * (1 - np.cos(2 * np.pi * (t - self.tbn) / (self.te - self.tbn)))

    def fk(self):
        pass
