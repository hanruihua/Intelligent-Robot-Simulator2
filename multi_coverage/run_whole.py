# this is an multi-agent coverage planing algo for building inspection
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: xinyi Wang
"""
import os
import sys

import numpy as np
import copy
import matplotlib.pyplot as plt
import time
from datetime import datetime

sys.path.append(os.path.dirname(__file__))

from initialize import ProblemSetup
from generate_grid_map import MapGenerate
from CSystem import CSystem
from cluster_centroid import cluster_centroid, pixel_centroid
from box2PolyVertsCons import box2PolyVertsCons
from env_plot_3d import EnvPlot3D


if __name__ == '__main__':
    # initialization
    nRobot = 3
    nBoxObs = 6

    prob = ProblemSetup(dim=3, robot_init=3, radius=0.2, maxSpeed=1.0, nRobot=nRobot, nBoxObs=nBoxObs)
    gmap = MapGenerate(boxVert=prob.boxVert, ws=prob.ws, xy_res=1, z_res=1, margin=1)
    ani = EnvPlot3D(ifShowSafeRegion=1,ifShowAnimation=1, save_ani=False, ws=prob.ws)
    # create a multi-robot system
    System = CSystem(pr=prob, gmap=gmap)

    System.initSystemState(prob)
    
    if ani.ifShowAnimation:
        color = ['r', 'g', 'b', 'm' , 'c', 'k', 'y']
        ax = ani.init_environment()
        # plt.grid(True)
        plt.ion()  # interactive mode on

        for iRobot in range(0,nRobot):
            ani.plot_robot(ax=ax,pos=System.MultiRobot_[iRobot].pos_real_,robot_color=color[iRobot])
    
        for jBoxObs in range(0,nBoxObs):
            obs_vert, __ = box2PolyVertsCons(prob.dim, prob.boxPos[:, jBoxObs], prob.boxSize[:, jBoxObs], prob.boxYaw[:, jBoxObs], prob.vert_m[jBoxObs])
            ani.plot_polyhedron(ax=ax,vert=obs_vert.T)

        if ani.ifShowSafeRegion:
            for iRobot in range(0,nRobot):
                pgon = System.MultiRobot_[iRobot].BVC_.verts
                ani.plot_polyhedron(ax=ax,vert=pgon,poly_color=color[iRobot],poly_alpha=0.1)

        # if ani.ifShowHistoryTra:
        #     robotHistoryTra_x = []
        #     robotHistoryTra_y = []
        #     robotHistoryTra_z = []
        #     for iRobot in range(0,nRobot):
        #         robotHistoryTra_x.append(System.MultiRobot_[iRobot].pos_real_[0,0])
        #         robotHistoryTra_y.append(System.MultiRobot_[iRobot].pos_real_[1,0])
        #         robotHistoryTra_z.append(System.MultiRobot_[iRobot].pos_real_[2,0])
        #         ax.plot3D(robotHistoryTra_x[iRobot],robotHistoryTra_y[iRobot],robotHistoryTra_z[iRobot],color=color[iRobot],linestyle = '-')

    logsize = 500
    # robot
    log_robot_pos_real = np.zeros((prob.dim, logsize, nRobot))
    log_robot_goal_current = np.zeros((prob.dim, logsize, nRobot))
    # log_robot_collision = np.zeros((1, logsize, nRobot))
    # log_robot_bvc = np.zeros((prob.dim*8, logsize, nRobot))

    n_loop = 0
    if_robots_arrived = np.zeros((nRobot, 1))

    while (n_loop < logsize):
        # time_start=time.time()

        if n_loop%10 ==0:
            now = datetime.now() # current date and time
            date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
            print(date_time,'Looping',n_loop,'\n')

        System.getSystemState()

        # logging state info
        for iRobot in range(0,nRobot):
            log_robot_pos_real[:, n_loop, iRobot] = System.MultiRobot_[iRobot].pos_real_[:,0]
            log_robot_goal_current[:, n_loop, iRobot] = System.MultiRobot_[iRobot].goal_projected_[:,0]
            # log_robot_bvc[n_loop, iRobot] = System.MultiRobot_[iRobot].BVC_.verts[:,0:8]

        if ani.ifShowAnimation == 1:
            plt.title('Iter:'+str(n_loop))

            fig_root = './log/'+str(n_loop) + '.png'
            if not os.path.exists(fig_root):
                os.mknod(fig_root)
            plt.savefig(fig_root)

            plt.cla()

            ax = ani.init_environment()
            for jBoxObs in range(0,nBoxObs):
                obs_vert, __ = box2PolyVertsCons(prob.dim, prob.boxPos[:, jBoxObs], prob.boxSize[:, jBoxObs], prob.boxYaw[:, jBoxObs], prob.vert_m[jBoxObs])
                ani.plot_polyhedron(ax=ax,vert=obs_vert.T)

            for iRobot in range(0,nRobot):
                ani.plot_robot(ax=ax,pos=System.MultiRobot_[iRobot].pos_real_,robot_color=color[iRobot])
            
            if ani.ifShowSafeRegion:
                for iRobot in range(0,nRobot):
                    pgon = System.MultiRobot_[iRobot].BVC_.verts
                    ani.plot_polyhedron(ax=ax,vert=pgon,poly_color=color[iRobot],poly_alpha=0.1)

            # if ani.ifShowHistoryTra:
            #     for iRobot in range(0,nRobot):
            #         # fig_robot_his_tra[iRobot].remove()
            #         # del fig_robot_his_tra[iRobot]
            #         # robotHistoryTra_x.append(System.MultiRobot_[iRobot].pos_real_[0,0])
            #         # robotHistoryTra_y.append(System.MultiRobot_[iRobot].pos_real_[1,0])
            #         # robotHistoryTra_z.append(System.MultiRobot_[iRobot].pos_real_[2,0])
            #         # fig_robot_his_tra.append(plt.plot(robotHistoryTra_x[iRobot],robotHistoryTra_y[iRobot],robotHistoryTra_z[iRobot],color=color[iRobot],linestyle = '-'))
            #         plt.plot(System.MultiRobot_[iRobot].pos_real_[0,0],
            #         System.MultiRobot_[iRobot].pos_real_[1,0],
            #         System.MultiRobot_[iRobot].pos_real_[2,0],color=color[iRobot],linestyle = '-')


            plt.pause(0.001)

        if np.sum(System.collision_mtx_) > 0:
            print('Collision happens!\n')
            break

        if n_loop == 96:
            print('debug!\n')

        # simulate one step
        System.simSystemOneStep()

        # if robot arrived
        for iRobot in range(0, nRobot):
            if_robots_arrived[iRobot] = System.MultiRobot_[iRobot].isArrived_

        # collision checking
        System.collisionChecking()

        if sum(if_robots_arrived) == nRobot:
            print('All robots arrived!\n')
            break

        n_loop = n_loop + 1

        # time_end=time.time()
        # print('time cost',time_end-time_start,'s')

    # STEP 2 Viewpoints generation
    for iRobot in range(0,nRobot):
        System.MultiRobot_[iRobot].computeBVC(useObs = 0)
        __, safe_region_pixel = pixel_centroid(gmap, System.MultiRobot_[iRobot].BVC_.verts)
        # pgon = System.MultiRobot_[iRobot].BVC_.verts
        # ani.plot_polyhedron(ax=ax,vert=pgon,poly_color=color[iRobot],poly_alpha=0.1)
        # print(safe_region_pixel.shape)

        ijk = np.where(safe_region_pixel!=0)
        vpt = gmap.idx_to_points(np.array(ijk).T)
        ptNum, centroids, labels, sub_vpt= cluster_centroid(vpt, gmap.nSub)
        System.MultiRobot_[iRobot].sub_region_viewpoint = sub_vpt
        System.MultiRobot_[iRobot].sub_region_ptNum = ptNum
        System.MultiRobot_[iRobot].sub_region_centroid = centroids
        

        if ani.ifShowAnimation:
            c2= ['green','yellow','purple']
            ax.scatter3D(centroids[:,0],centroids[:,1],centroids[:,2],
            c=color[iRobot],s=50,label="Centers",alpha=1,marker='^')
            
            for i in range(0,gmap.nSub):
                li = np.array(labels==i)
                ax.scatter3D(vpt[li,0],vpt[li,1],vpt[li,2],c=c2[i],s=1,label="C1",alpha=0.2)
            plt.pause(0.001)

    # plt.ioff()
    # plt.show()

    for iRobot in range(0,nRobot):
        np.savetxt('./log/robot_pos_'+str(iRobot)+'.txt', log_robot_pos_real[:,:,iRobot],fmt='%f',delimiter='\t')
        # file = open(os.path.dirname(__file__)+'/log/robot_bvc_'+str(iRobot)+'.txt','w')
        # file.write(str(log_robot_bvc[iRobot]))
        # file.close()

    print('done')
