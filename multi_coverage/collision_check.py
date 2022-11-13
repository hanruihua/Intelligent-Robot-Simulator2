import os
import numpy as np
import openGJK_cython as opengjk

def collision_check(nRobot = None,nBoxObs = None,robot_pos = None,robot_radius = None,box_vert = None): 
    
    # Check if collision happends among multiple agents with obstacles
    
    dim = robot_pos.shape[0]
    ## inter robot
    collision_mtx_robot = np.zeros((nRobot,nRobot))
    for i in range(0,nRobot-1):
        for j in range(i+1,nRobot):
            pos_ij = robot_pos[:,i] - robot_pos[:,j]
            size_ij = 2 * robot_radius
            d_ij = np.linalg.norm(pos_ij)
            if d_ij < size_ij * 1:
                collision_mtx_robot[i,j] = 1
            collision_mtx_robot[j,i] = collision_mtx_robot[i,j]
    
    ## robot obstacle
    collision_mtx_obs = np.zeros((nRobot,nBoxObs))
    for i in range(0,nRobot):
        for j in range(0,nBoxObs):
            pos_i = robot_pos[:,i]
            # vert_j = box_vert[:,:,j]
            vert_j = box_vert[0,j]
            if dim == 2:
                dis_ij = p_poly_dist(pos_i[0],pos_i[1],vert_j[1,:],vert_j[2,:])
            else:
                if dim == 3:
                    dis_ij = opengjk.pygjk(pos_i.T,vert_j.T)
                else:
                    raise Exception('Dimension error!')
            if dis_ij < robot_radius * 1:
                collision_mtx_obs[i,j] = 1
    
    collision_mtx = np.hstack((collision_mtx_robot, collision_mtx_obs))

    return collision_mtx