import numpy as np
from hyperplane import boundsCon
from artools.artools import vert2con, con2vert


class BVC():
    def __init__(self, pr):
        self.Ab_bound = np.zeros((6, pr.dim+1))
        self.idx_robot = []

        self.Ab_robot = []

        self.idx_obs = []
        self.Ab_obs = []

        self.Ab = np.zeros((6, pr.dim+1))

        # initial safe region
        A_bound, b_bound = boundsCon(pr.dim, pr.ws[:, 0], pr.ws[:, 1])

        self.Ab = np.hstack((A_bound, b_bound))
        self.verts = con2vert(A_bound, b_bound)

