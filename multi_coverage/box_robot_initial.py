import numpy as np
import os


def robotStartPos_2D(nRobot=None, mode=None, xDim=None, yDim=None, radius=None):
    robotStartPos = np.zeros((2, nRobot))
    diameter = 2.0 * radius

    if 1 == mode:
        angle = 2.0 * np.pi / nRobot
        R = 0.4 * (xDim(2) - xDim(1))
        for iRobot in np.arange(1, nRobot + 1).reshape(-1):
            angle_i = deg2rad(0) + angle * (iRobot - 1)
            robotStartPos[np.arange[1, 2 + 1], iRobot] = np.array([[R * np.cos(angle_i)], [R * np.sin(angle_i)]])
    else:
        if 2 == mode:
            idx_data = scipy.io.loadmat('idx_valid_10.mat')
            idx_valid = idx_data.xy_idx_valid
            N_idx = idx_valid.shape[2 - 1]
            rand_idx = randperm(N_idx)
            for iRobot in np.arange(1, nRobot + 1).reshape(-1):
                robotStartPos[1, iRobot] = 0.8 * idx_valid(1, rand_idx(iRobot)) - 5
                robotStartPos[2, iRobot] = 0.8 * idx_valid(2, rand_idx(iRobot)) - 5
        else:
            if 3 == mode:
                angle = 2.0 * np.pi / nRobot
                ang_gap = 0.5 * angle
                R_min = 0.3
                R_max = xDim(2) - 0.3
                for iRobot in np.arange(1, nRobot + 1).reshape(-1):
                    # initial
                    angle_c = deg2rad(2 * np.pi * rand) + angle * (iRobot - 1)
                    angle_min = angle_c - ang_gap
                    angle_max = angle_c + ang_gap
                    angle_i = angle_min + (angle_max - angle_min) * rand
                    R_i = R_min + (R_max - R_min) * rand
                    robotStartPos[np.arange[1, 2 + 1], iRobot] = np.array(
                        [[R_i * np.cos(angle_i)], [R_i * np.sin(angle_i)]])
            else:
                raise Exception('Robot starting positions initialization failed!')

    return robotStartPos[:,0:nRobot]


def robotStartPos_3D(nRobot=5, mode=1, xDim=None, yDim=None, zDim=None, radius=0.1):
    robotStartPos = np.zeros((3, 50))
    diameter = 2 * radius

    if mode == 0:
        x = np.linspace(xDim[0] + 1, xDim[1] - 4, nRobot)  # line
        y = yDim[0] + 1
        for iRobot in range(0, nRobot):
            xy = np.row_stack((x[iRobot], y))
            robotStartPos[0:2, iRobot] = xy[:,0]
            robotStartPos[2, iRobot] = zDim[0] + 0.5 * (zDim[1] - zDim[0])
    elif mode == 1:  # uniformly distributed in a circle
        angle = 2.0 * np.pi / nRobot
        R = 0.1 * (xDim[1] - xDim[0])
        for iRobot in range(0, nRobot):
            angle_i = np.deg2rad(0) + angle * iRobot
            robotStartPos[0:2, iRobot] = np.hstack((R * np.cos(angle_i)+xDim[0]/2.0, R * np.sin(angle_i)+xDim[0]/2.0))
            robotStartPos[2, iRobot] = zDim[0] + 0.5 * (zDim[1] - zDim[0])
    elif mode == 2:  # complete random position
        pass
        # # discrete the workspace
        xL = xDim[1] - xDim[0]
        yL = yDim[1] - yDim[0]
        zL = zDim[1] - zDim[0]
        xN = int(np.floor(xL / diameter)) - 2
        yN = int(np.floor(yL / diameter)) - 2
        zN = int(np.floor(zL / diameter)) - 2
        # generate random integer number
        for iRobot in np.arange(1, nRobot + 1).reshape(-1):
            xN_i = randi(np.array([1, xN - 1]), 1)
            yN_i = randi(np.array([1, yN - 1]), 1)
            zN_i = randi(np.array([1, zN - 1]), 1)
            robotStartPos[0, iRobot] = xDim[0] + diameter * xN_i
            robotStartPos[1, iRobot] = yDim[0] + diameter * yN_i
            robotStartPos[2, iRobot] = zDim[0] + diameter * zN_i
    elif mode == 3:
        robotStartPos[:,0] = [-9,-9,5]
        robotStartPos[:,1] = [-5,-12,5]
        robotStartPos[:,2] = [ 0,-9,5]
        robotStartPos[:,3] = [ 0 ,-12,5]
        robotStartPos[:,4] = [ 2,-9,5]

        # robotStartPos[:,0] = [-30,-0,5]
        # robotStartPos[:,1] = [-30,-5,5]
        # robotStartPos[:,2] = [-30,-11,5]

        # robotStartPos[:,0] = [-30,-0,5]
        # robotStartPos[:,1] = [-30,-10,5]
        # robotStartPos[:,2] = [-25,-11,5]

    else:
        raise Exception('Robot starting positions initialization failed!')

    return robotStartPos[:,0:nRobot]


def box_initial_2D(num_box):
    box_pos = np.zeros((2, nBoxObs))
    box_size = np.zeros((2, nBoxObs))
    box_yaw = np.zeros((1, nBoxObs))
    if num_box == 0:
        nBoxObs = 0
        box_pos = []
        box_size = []
        box_yaw = []
    elif num_box == 1:
        box_pos[:, 1] = np.array([[- 4], [1]])
        box_size[:, 1] = np.array([[3], [2]])
        box_yaw[:, 1] = deg2rad(0)
       

    return box_pos[:,0:num_box], box_size[:,0:num_box], box_yaw[:,0:num_box]


def box_initial_3D(num_box):
    box_pos = np.zeros((3, 50))
    box_size = np.zeros((3, 50))
    box_yaw = np.zeros((1, 50))
    if num_box == 0:
        box_pos = []
        box_size = []
        box_yaw = []
    elif num_box == 1:
        # box_pos[:, 0] = np.array([0, 1, 4])
        # box_size[:, 0] = np.array([3, 2, 8])
        box_pos[:, 0] = np.array([0, 0, 8])
        box_size[:, 0] = np.array([52, 14, 16])
        box_yaw[:, 0] = np.deg2rad(0)

    elif num_box == 2:
        # box_pos[:, 0]  = np.array([0, 1, 2])
        box_pos[:, 0] = np.array([0, 1, 2])
        box_size[:, 0] = np.array([3, 2, 10])
        box_yaw[:, 0] = np.deg2rad(0)
        box_pos[:, 1] = np.array([0, 4, 3])
        box_size[:, 1] = np.array([1.5, 1, 3.0])
        box_yaw[:, 1] = np.deg2rad(0)
    elif num_box == 3:
        box_pos[:, 0] = np.array([0, 0, 8])
        box_size[:, 0] = np.array([52, 14, 16])
        box_yaw[:, 0] = np.deg2rad(0)

        box_pos[:, 1] = np.array([-26.0, -13.0, 4.0])
        box_size[:, 1] = np.array([4.5, 3.0, 18.0])
        box_yaw[:, 1] = np.deg2rad(-45)

        box_pos[:, 2] = np.array([30.0, -15, 5.0])
        box_size[:, 2] = np.array([2.6, 3.2, 10.0])
        box_yaw[:, 2] = np.deg2rad(0)
    elif num_box == 6:
        box_pos[:, 0] = np.array([0, 0, 8])
        box_size[:, 0] = np.array([52, 14, 16])
        box_yaw[:, 0] = np.deg2rad(0)

        box_pos[:, 1] = np.array([-35.0, -20.0, 5.0])
        box_size[:, 1] = np.array([4.5, 3.0, 10.0])
        box_yaw[:, 1] = np.deg2rad(-45)

        box_pos[:, 2] = np.array([30.0, -17, 2.0])
        box_size[:, 2] = np.array([4.6, 5.2, 4.0])
        box_yaw[:, 2] = np.deg2rad(0)

        box_pos[:, 3] = np.array([30.0, 20, 3.0])
        box_size[:, 3] = np.array([6.0, 8.2, 6.0])
        box_yaw[:, 3] = np.deg2rad(-30)

        box_pos[:, 4] = np.array([0.0, 20, 5.0])
        box_size[:, 4] = np.array([24.0, 5.2, 10.0])
        box_yaw[:, 4] = np.deg2rad(0)

        box_pos[:, 5] = np.array([-20.0, -15, 1.0])
        box_size[:, 5] = np.array([9.0, 6.2, 2.0])
        box_yaw[:, 5] = np.deg2rad(60)
    else:
        raise Exception('Obstacle positions initialization failed!')

    return box_pos[:,0:num_box], box_size[:,0:num_box], box_yaw[:,0:num_box]
