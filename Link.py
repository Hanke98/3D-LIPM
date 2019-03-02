import numpy as np
import glm


class Link:
    def __init__(self, end, theta):
        self.end = glm.vec4(end)
        self.theta = theta
        self.trans = end[:-1]
