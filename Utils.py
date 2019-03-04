import glm

import numpy as np

x_axis = glm.vec3([1, 0, 0])
y_axis = glm.vec3([0, 1, 0])
z_axis = glm.vec3([0, 0, 1])


def to_numpy(m):
    if isinstance(m, glm.tvec4):
        a = [d for d in m][:-1]
    else:
        a = [[v[0], v[1], v[2], v[3]] for v in m]
    return np.array(a)


def rotate(m, theta):
    m = glm.rotate(m, glm.radians(theta[0]), x_axis)
    m = glm.rotate(m, glm.radians(theta[1]), y_axis)
    m = glm.rotate(m, glm.radians(theta[2]), z_axis)
    return m
