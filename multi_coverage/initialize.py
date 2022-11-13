from box_robot_initial import box_initial_3D, box_initial_2D, robotStartPos_3D, robotStartPos_2D
from box2PolyVertsCons import box2PolyVertsCons
from collision_check import collision_check
import numpy as np
import copy

class ProblemSetup:
    def __init__(self, dim=3, robot_init=0, radius=0.2, maxSpeed=1.0, nRobot=5, nBoxObs=1):
        # environment dimension
        self.dim = dim
        self.xdim = np.array([-40, 40])
        self.ydim = np.array([-25, 25])
        self.zdim = np.array([0, 16])

        if self.dim == 2:
            self.ws = np.array([self.xdim, self.ydim])
        elif self.dim == 3:
            # self.zdim = np.array([0, 10])
            self.ws = np.array([self.xdim, self.ydim, self.zdim])

        # robot physics

        self.robot_type = 0
        
        self.radius = radius
        # robot maximal speed
        self.maxSpeed = maxSpeed
        # sensor range
        self.boundLocalRadius = 100.0
        # simulation setup
        self.dtSim = 0.5
        self.control = 1

        if self.control == 0:
            print('Controller mode: project point. \n' % ())
        elif self.control == 1:
            print('Controller mode: move to centroid. \n' % ())
        else:
            raise Exception('Controller mode is not defined!')


        # Scenario setup
        # static obstacles
        self.nBoxObs = nBoxObs
        self.vert_m = np.ones((nBoxObs,1))*16
        if self.dim == 2:
            self.vert_m[0] = 4
            boxPos, boxSize, boxYaw = box_initial_2D(nBoxObs)
        elif self.dim == 3:
            self.vert_m[0] = 16
            boxPos, boxSize, boxYaw = box_initial_3D(nBoxObs)

        boxVert = np.empty((1,nBoxObs),dtype=object)
        for iBox in range(0, nBoxObs):
            temp_vert, temp_Ab = box2PolyVertsCons(self.dim, boxPos[:, iBox], boxSize[:, iBox], boxYaw[:, iBox], self.vert_m[iBox])
            boxVert[0,iBox] = temp_vert

        # multiple robot, robot initial and end position should not collide with each other and with obstacles
        self.nRobot = nRobot

        collision_mtx = np.ones((nRobot, nRobot + nBoxObs))
        
        while (np.sum(collision_mtx) > 0):
            print('Generating robot initial positions and goals ... \n' % ())
            if self.dim == 2:
                robotStartPos = robotStartPos_2D(nRobot, 3, self.ws[0, :], self.ws[1, :], self.radius)
                robotEndPos = - robotStartPos
            elif self.dim == 3:
                robotStartPos = robotStartPos_3D(nRobot, robot_init, self.ws[0, :], self.ws[1, :], self.ws[2, :], self.radius)
                robotEndPos = -copy.deepcopy(robotStartPos)
                robotEndPos[2, :] = self.zdim[0] + self.zdim[1] - robotStartPos[2, :]
                collision_mtx_start = collision_check(nRobot, nBoxObs, robotStartPos, 1.2 * self.radius, boxVert)
                collision_mtx_end = collision_check(nRobot, nBoxObs, robotEndPos, 1.2 * self.radius, boxVert)
                collision_mtx = np.array([[collision_mtx_start], [collision_mtx_end]])
                collision_mtx = np.zeros((nRobot, nRobot + nBoxObs))

        self.robotStartPos = robotStartPos
        self.robotEndPos = robotEndPos
        self.boxPos = boxPos
        self.boxSize = boxSize
        self.boxYaw = boxYaw
        self.boxVert = boxVert
