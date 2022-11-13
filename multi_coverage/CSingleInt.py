import numpy as np
from generate_grid_map import MapGenerate
from BVC import BVC
from LocalInfo import LocalInfo
from hyperplane import boundsCon, point_point_hyperplane, point_box_shifted_hyperplane, point_polytope_shifted_hyperplane
from artools.artools import vert2con, con2vert
from cluster_centroid import pixel_centroid

import copy


class CSingleInt():
    # Class for single-integrator robots
    def __init__(self, pr, gmap, robotID):
        self.dim_ = pr.dim
        self.id_ = robotID
        self.radius_ = pr.radius
        self.radius_bvc_ = 1.0 * pr.radius
        self.dt_ = pr.dtSim
        self.speed_max_ = pr.maxSpeed

        self.vert_m = pr.vert_m
        
        # self.useObs_ = 1
        
        self.isCollision_ = 0

        self.isArrived_ = 0

        self.BVC_ = BVC(pr)
        # self.safe_region_ = np.array([])
        self.robots_info_local_ = LocalInfo()
        self.obs_info_local_ = LocalInfo()

        self.centroid = np.zeros((self.dim_, 1))
        self.safe_region_pixel = []

        self.bound_radius_ = pr.boundLocalRadius
        self.bound_global_ = copy.deepcopy(pr.ws)
        self.bound_local_ = copy.deepcopy(pr.ws)

        # vector and matrix initialization
        self.pos_real_ = np.zeros((self.dim_, 1))
        self.u_ = np.zeros((self.dim_, 1))
        self.goal_final_ = np.zeros((self.dim_, 1))
        self.goal_current_ = np.zeros((self.dim_, 1))
        self.goal_projected_ = np.zeros((self.dim_, 1))
        # tolerance for reaching the goal
        self.goal_tolerance_ = 0.05

        # lloyds
        self.centroid = np.zeros((self.dim_, 1))
        self.safe_region_pixel = np.zeros((gmap.nx, gmap.ny, gmap.nz), dtype=bool)
        self.sub_region_viewpoint = np.transpose(gmap.obs_free_pt)
        self.sub_region_ptNum = np.zeros((gmap.nSub, 1))
        self.sub_region_centroid = np.zeros((self.dim_, gmap.nSub))

        ## ========== Compute local bound ==========

    def computeLocalBound(self):
        for i in range(0, self.dim_):
            lb = self.pos_real_[i] - self.bound_radius_
            ub = self.pos_real_[i] + self.bound_radius_
            self.bound_local_[i, 0] = max(lb, self.bound_global_[i, 0])
            self.bound_local_[i, 1] = min(ub, self.bound_global_[i, 1])

        ## ========== If an object in local bound ==========

    def isPointInLocalBound(self, pt):
        inside = 1
        for i in range(0, self.dim_):
            if pt[i] < self.bound_local_[i, 0] or pt[i] > self.bound_local_[i, 1]:
                inside = 0

        return inside

        ## ========== Get local robots info ==========

    def getLocalRobotInfo(self, multi_robot_pos_real_):
        
        self.robots_info_local_.idx = []
        self.robots_info_local_.pos = []
        
        nRobot = multi_robot_pos_real_.shape[1]
        k = 0
        for iRobot in range(0, nRobot):
            if iRobot == self.id_:
                continue
            pt = multi_robot_pos_real_[:, iRobot]
            inside = self.isPointInLocalBound(pt)
            if inside:
                # self.robots_info_local_.idx[k] = iRobot
                # self.robots_info_local_.pos[:, k] = multi_robot_pos_real_[:, iRobot]
                self.robots_info_local_.idx.append(iRobot)
                self.robots_info_local_.pos.append(multi_robot_pos_real_[:, iRobot])
                k = k + 1

        ## ========== Get local obs info ==========

    def getLocalObsInfo(self, multi_obs_pos_real_, multi_obs_size, multi_obs_yaw):
        self.obs_info_local_.idx = []
        self.obs_info_local_.pos = []
        self.obs_info_local_.yaw = []
        self.obs_info_local_.size = []

        nBoxObs = multi_obs_pos_real_.shape[1]
        k = 0
        for jBoxObs in range(0, nBoxObs):
            pt = multi_obs_pos_real_[:, jBoxObs]
            inside = self.isPointInLocalBound(pt)
            if inside:
                self.obs_info_local_.idx.append(jBoxObs)
                self.obs_info_local_.pos.append(multi_obs_pos_real_[:, jBoxObs])
                self.obs_info_local_.size.append(multi_obs_size[:, jBoxObs])
                self.obs_info_local_.yaw.append(multi_obs_yaw[:, jBoxObs])
                k = k + 1

        ## ========== Compute BVC ==========

    def computeBVC(self, control=1, useObs = 1):
        self.BVC_.Ab_bound = np.zeros((6, self.dim_+1))

        self.BVC_.idx_robot = []
        self.BVC_.Ab_robot = []

        self.BVC_.idx_obs = []
        self.BVC_.Ab_obs = []
        
        self.BVC_.Ab = []
        self.BVC_.verts = []

        r = self.radius_bvc_
        pe = self.pos_real_[:,0]

        # local bound
        A_bound, b_bound = boundsCon(self.dim_, self.bound_local_[:, 0], self.bound_local_[:, 1])
        self.BVC_.Ab_bound[:,0:self.dim_] = A_bound
        self.BVC_.Ab_bound[:,self.dim_] = b_bound.reshape(A_bound.shape[0]) - r * np.linalg.norm(A_bound,axis=1)
      
        # loop for each other local robot
        for i in range(0,len(self.robots_info_local_.pos)):
            pt = self.robots_info_local_.pos[i]
            a,b = point_point_hyperplane(pe, pt)
            self.BVC_.idx_robot.append(self.robots_info_local_.idx[i])
            ab = np.hstack((np.transpose(a), np.array([b - r * np.linalg.norm(np.transpose(a))]) ))
            self.BVC_.Ab_robot.append(ab)

        # loop for each local obs
        for i in range(0,len(self.obs_info_local_.pos)):
            obs_pt = self.obs_info_local_.pos[i]
            obs_size = self.obs_info_local_.size[i]
            obs_yaw = self.obs_info_local_.yaw[i]
            a,b = point_box_shifted_hyperplane(pe, obs_pt, obs_size, obs_yaw, self.vert_m[i])
            self.BVC_.idx_obs.append(self.obs_info_local_.idx[i])
            ab = np.hstack((a.T, np.array([b - r * np.linalg.norm(np.transpose(a))]) ))
            self.BVC_.Ab_obs.append(ab)

        # combine Ab
        if control == 0:
            self.BVC_.Ab = np.row_stack((self.BVC_.Ab_bound,self.BVC_.Ab_robot,self.BVC_.Ab_obs))
        elif control== 1:
            if useObs == 0:
                self.BVC_.Ab = np.row_stack((self.BVC_.Ab_bound,self.BVC_.Ab_robot))
            elif useObs == 1:
                self.BVC_.Ab = np.row_stack((self.BVC_.Ab_bound,self.BVC_.Ab_robot,self.BVC_.Ab_obs))

        # convert to verts
        test_temp = np.dot(self.BVC_.Ab[:, 0:self.dim_], pe) - self.BVC_.Ab[:,self.dim_]
        if not self.isCollision_ and not sum(test_temp > 0):
            self.BVC_.verts = con2vert(self.BVC_.Ab[:,0:self.dim_],self.BVC_.Ab[:,self.dim_])
        else:
            self.BVC_.verts = []

        ## ========== Compute move to centroid policy ==========

    def computeControlInput1(self):
        self.u_ = np.zeros((self.dim_, 1))
        # move to centroid
        distance_to_centroid = np.linalg.norm(self.goal_projected_ - self.pos_real_)
        if distance_to_centroid <= self.goal_tolerance_:
            self.u_ = np.zeros((self.dim_, 1))
        else:
            # moving direction
            vel_direction = (self.goal_projected_ - self.pos_real_) / distance_to_centroid
            # moving speed
            vel_speed = min(self.speed_max_, distance_to_centroid / self.dt_)
            # optimal velocity
            self.u_ = vel_speed * vel_direction

        ## ========== Choose next movement ==========

    def chooseNextMove(self, gmap=None):
        self.centroid = []
        self.safe_region_pixel = []

        boxSize = self.obs_info_local_.size
        self.centroid, self.safe_region_pixel = pixel_centroid(gmap, self.BVC_.verts)

        # if centroid goal inside obstacle
        centroid_id = gmap.points_to_idx(np.transpose(self.centroid))
        isCollision = gmap.occ_map[centroid_id[0,0], centroid_id[0,1], centroid_id[0,2]]
        if isCollision:
            pass
            self.goal_projected_ = self.centroid
            # TODO
            proj_centroid = pso_select(gmap,np.transpose(self.pos_real_),np.transpose(self.centroid),boxSize)
            self.goal_projected_ = np.transpose(proj_centroid)
        else:
            self.goal_projected_ = self.centroid

        ## ========== Reaching goal checking ==========

    def ReachGoalChecking(self=None):
        if not self.isArrived_ and np.linalg.norm(self.u_) < 0.05 * self.speed_max_:
            self.isArrived_ = 1
        else:
            if np.linalg.norm(self.u_) > 0.05 * self.speed_max_:
                self.isArrived_ = 0

        ## ========== Simulate one step ==========

    def simulateOneStep(self=None):
        # perform one-step simulation for the robot
        dpos = self.u_ * self.dt_

        self.pos_real_ = self.pos_real_ + dpos
