from CSingleInt import CSingleInt
from CBoxObs import CBoxObs
from box2PolyVertsCons import box2PolyVertsCons
from collision_check import collision_check
import time
import numpy as np

class CSystem():
    # Class for a multi-robot system
    def __init__(self, pr, gmap):

        self.dim_ = pr.dim
        self.dt_ = pr.dtSim
        self.ws_ = pr.ws
        self.gmap = gmap

        self.time_global_ = 0
        self.time_step_global_ = 0

        self.nRobot_ = pr.nRobot
        self.nBoxObs_ = pr.nBoxObs

        self.control_ = pr.control

        self.MultiRobot_ = []
        self.MultiBoxObs_ = []

        for iRobot in range(0, self.nRobot_):
            if pr.robot_type == 0:
                self.MultiRobot_.append(CSingleInt(pr, gmap, iRobot))
            else:
                raise Exception('Robot type not defined!')
        
        for jBoxObs in range(0, self.nBoxObs_):
            self.MultiBoxObs_.append(CBoxObs(pr, jBoxObs))

        # vector and matrix initialization
        self.vert_m = pr.vert_m 

        self.multi_robot_pos_real_ = np.zeros((self.dim_,self.nRobot_))
        self.multi_robot_goal_real_ = np.zeros((self.dim_,self.nRobot_))
        self.multi_obs_pos_real_ = np.zeros((self.dim_,self.nBoxObs_))
        self.multi_obs_size_real_ = np.zeros((self.dim_,self.nBoxObs_))
        self.multi_obs_yaw_real_ = np.zeros((1,self.nBoxObs_))
        self.collision_mtx_ = np.zeros((self.nRobot_,self.nRobot_ + self.nBoxObs_))
        self.multi_obs_vert_real_ = np.empty((1,self.nBoxObs_),dtype=object)


    def initSystemState(self,pr):
        for iRobot in range(0,self.nRobot_):
            self.MultiRobot_[iRobot].pos_real_[:,0] = pr.robotStartPos[:, iRobot]
            #goal
            self.MultiRobot_[iRobot].goal_final_[:,0] = pr.robotEndPos[:, iRobot]
            self.MultiRobot_[iRobot].goal_current_[:,0] = pr.robotEndPos[:, iRobot]

            # box obs initial state
        for jBoxObs in range(0,self.nBoxObs_):
            self.MultiBoxObs_[jBoxObs].pos_real_[:,0] = pr.boxPos[:, jBoxObs]
            self.MultiBoxObs_[jBoxObs].size_[:,0] = pr.boxSize[:, jBoxObs]
            self.MultiBoxObs_[jBoxObs].yaw_ = pr.boxYaw[:, jBoxObs]

    def getSystemState(self):
        for iRobot in range(0,self.nRobot_):
            self.multi_robot_pos_real_[:,iRobot] = self.MultiRobot_[iRobot].pos_real_[:,0]
            self.multi_robot_goal_real_[:,iRobot] = self.MultiRobot_[iRobot].goal_final_[:,0]
        
        for jBoxObs in range(0,self.nBoxObs_):
            self.multi_obs_pos_real_[:,jBoxObs] = self.MultiBoxObs_[jBoxObs].pos_real_[:,0]
            self.multi_obs_size_real_[:,jBoxObs] = self.MultiBoxObs_[jBoxObs].size_[:,0]
            self.multi_obs_yaw_real_[:,jBoxObs] = self.MultiBoxObs_[jBoxObs].yaw_
            self.multi_obs_vert_real_[0,jBoxObs], __ = box2PolyVertsCons(self.dim_, self.multi_obs_pos_real_[:, jBoxObs],
                                                                       self.multi_obs_size_real_[:, jBoxObs],
                                                                       self.multi_obs_yaw_real_[:, jBoxObs],
                                                                       self.vert_m[jBoxObs])        

    def simSystemOneStep(self):
        # Simulate the system for one step, given current system state
        self.time_step_global_ = self.time_step_global_ + 1
        
        for iRobot in range(0,self.nRobot_):
            time_start=time.time()

            # ===== compute local bound =====
            self.MultiRobot_[iRobot].computeLocalBound()
            # ===== obtain local robot info =====
            self.MultiRobot_[iRobot].getLocalRobotInfo(self.multi_robot_pos_real_)
            # ===== obtain local box obs info =====
            self.MultiRobot_[iRobot].getLocalObsInfo(self.multi_obs_pos_real_,self.multi_obs_size_real_,self.multi_obs_yaw_real_)
            # ===== compute local safe convex region hyperplanes =====
            self.MultiRobot_[iRobot].computeBVC(self.control_)
            if self.control_ == 0:
                pass
            elif self.control_ == 1:
                # ===== choose next movement using lloyds =====
                self.MultiRobot_[iRobot].chooseNextMove(self.gmap)
                # ===== move to centroid =====
                self.MultiRobot_[iRobot].computeControlInput1()
                # ===== reaching goal checking =====
                self.MultiRobot_[iRobot].ReachGoalChecking()
            # if arrived, set to 0
            if self.MultiRobot_[iRobot].isArrived_ == 1:
                self.MultiRobot_[iRobot].u_ = 0 * self.MultiRobot_[iRobot].u_
            # if in collision, set to 0
            if self.MultiRobot_[iRobot].isCollision_ >= 1:
                self.MultiRobot_[iRobot].u_ = 0 * self.MultiRobot_[iRobot].u_
                print('Robot %d in collision!\n' % (iRobot))

            # ===== simulate the robot one step =====
            self.MultiRobot_[iRobot].simulateOneStep()
        
            time_end=time.time()
            print('time cost',time_end-time_start,'s')



    def collisionChecking(self):
        # Check if collision happens in the system
        self.collision_mtx_ = collision_check(self.nRobot_,self.nBoxObs_,self.multi_robot_pos_real_,self.MultiRobot_[0].radius_,self.multi_obs_vert_real_)
        for iRobot in range(0,self.nRobot_):
            if self.MultiRobot_[iRobot].isCollision_ == 0:
                self.MultiRobot_[iRobot].isCollision_ = min(1,sum(self.collision_mtx_[iRobot,:]))        