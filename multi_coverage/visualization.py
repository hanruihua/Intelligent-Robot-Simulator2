import os
import sys
sys.path.append(os.path.dirname(__file__))

from env_plot_3d import EnvPlot3D
import matplotlib.pyplot as plt
from initialize import ProblemSetup
from box2PolyVertsCons import box2PolyVertsCons
import numpy as np

if __name__ == '__main__':

    nRobot = 3
    nBoxObs = 1

    logsize = 500
    n_loop = 0

    prob = ProblemSetup(dim=3, robot_type=0, radius=0.2, maxSpeed=1.0, nRobot=nRobot, nBoxObs=nBoxObs)

    robot_pos = np.zeros((prob.dim,logsize,nRobot))
    for iRobot in range(0,nRobot):
        robot_pos[:,:,iRobot] = np.loadtxt(os.path.dirname(__file__)+'/log/robot_pos_'+str(iRobot)+'.txt') 


    ani = EnvPlot3D(ifShowSafeRegion=0,ifShowAnimation=0, save_ani=False, ws=prob.ws)

    color = ['r', 'g', 'b', 'm' , 'c', 'k', 'y']

    ax = ani.init_environment()
    plt.grid(True)
    plt.ion()  # interactive mode on

    for jBoxObs in range(0,nBoxObs):
        obs_vert, __ = box2PolyVertsCons(prob.dim, prob.boxPos[:, jBoxObs], prob.boxSize[:, jBoxObs], prob.boxYaw[:, jBoxObs], prob.vert_m[jBoxObs])
        ani.plot_polyhedron(ax=ax,vert=obs_vert.T)

    # fig_robot_his_tra = []
    # if ani.ifShowHistoryTra:
    #     robotHistoryTra_x = []
    #     robotHistoryTra_y = []
    #     robotHistoryTra_z = []
    #     for iRobot in range(0,nRobot):
    #         robotHistoryTra_x.append(System.MultiRobot_[iRobot].pos_real_[0,0])
    #         robotHistoryTra_y.append(System.MultiRobot_[iRobot].pos_real_[1,0])
    #         robotHistoryTra_z.append(System.MultiRobot_[iRobot].pos_real_[2,0])
    #         fig = ax.plot3D(robotHistoryTra_x[iRobot],robotHistoryTra_y[iRobot],robotHistoryTra_z[iRobot],color=color[iRobot],linestyle = '-')
    #         fig_robot_his_tra.append(fig)

    # logsize = robot_pos.shape[0]

    while (n_loop < logsize):
        plt.cla()

        ax = ani.init_environment()
        for jBoxObs in range(0,nBoxObs):
            obs_vert, __ = box2PolyVertsCons(prob.dim, prob.boxPos[:, jBoxObs], prob.boxSize[:, jBoxObs], prob.boxYaw[:, jBoxObs], prob.vert_m[jBoxObs])
            ani.plot_polyhedron(ax=ax,vert=obs_vert.T)

        for iRobot in range(0,nRobot):
            ani.plot_robot(ax=ax,pos=robot_pos[:,n_loop,iRobot],robot_color=color[iRobot])
        if ani.ifShowSafeRegion:
            for iRobot in range(0,nRobot):
                # need to do replace this pgon 
                # pgon = System.MultiRobot_[iRobot].BVC_.verts  
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
        n_loop = n_loop + 1
        
        plt.pause(0.001)

    plt.ioff()
    plt.show()

    print('done')


    plt.show()





