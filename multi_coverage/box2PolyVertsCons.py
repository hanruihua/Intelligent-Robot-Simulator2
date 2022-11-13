# transform a box to polytope verts and linear constraints
#
# inputs:
#   - dim: dimension
#   - box_pos: box center position, x and y, [dx1]
#   - box_size: box box_size, length and width, [dx1]
#   - orientation_xy: box orientation, [1]
#
# outputs:
#   - poly_vert: vertice represented the box, d*m
#   - poly_Ab: linear constraints represented half planes of the box, (d+1)*m
#
import numpy as np
import math
import copy
import os
import sys
from artools.artools import vert2con, con2vert
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull

def box2PolyVertsCons(dim=3, box_pos=None, box_size=None, orientation_xy=None,vert_m=None):
    if dim == 2:
        poly_vert, poly_Ab = box2PolyVertsCons_2D(box_pos, box_size, orientation_xy)
    elif dim == 3:
        if vert_m == 8:
            poly_vert, poly_Ab = box2PolyVertsCons_3D(box_pos, box_size, orientation_xy)
        else:
            poly_vert, poly_Ab = box2EllipseVertsCons_3D(box_pos, box_size, orientation_xy,vert_m)
    else:
        raise Exception('Dimension error!')
    return poly_vert, poly_Ab


def box2PolyVertsCons_3D(box_pos=None, box_size=None, orientation_xy=None):
    #     poly_vert = zeros(3, 8);            # 8 vert
    #     poly_Ab   = zeros(4, 6);            # 6 faces

    ## vertices
    # before rotation and translation
    x_min = - 0.5 * box_size[0]
    x_max = + 0.5 * box_size[0]
    y_min = - 0.5 * box_size[1]
    y_max = + 0.5 * box_size[1]
    z_min = - 0.5 * box_size[2]
    z_max = + 0.5 * box_size[2]

    p1 = np.array([x_min, y_min, z_min])
    p2 = np.array([x_max, y_min, z_min])
    p3 = np.array([x_max, y_max, z_min])
    p4 = np.array([x_min, y_max, z_min])
    p5 = np.array([x_min, y_min, z_max])
    p6 = np.array([x_max, y_min, z_max])
    p7 = np.array([x_max, y_max, z_max])
    p8 = np.array([x_min, y_max, z_max])

    # only rotate in yaw (xy plane), then translation
    rot_mtx = rotz(np.rad2deg(orientation_xy))
    p1_r = np.dot(rot_mtx, p1) + box_pos
    p2_r = np.dot(rot_mtx, p2) + box_pos
    p3_r = np.dot(rot_mtx, p3) + box_pos
    p4_r = np.dot(rot_mtx, p4) + box_pos
    p5_r = np.dot(rot_mtx, p5) + box_pos
    p6_r = np.dot(rot_mtx, p6) + box_pos
    p7_r = np.dot(rot_mtx, p7) + box_pos
    p8_r = np.dot(rot_mtx, p8) + box_pos

    # polytope verts
    poly_vert = np.array([p1_r, p2_r, p3_r, p4_r, p5_r, p6_r, p7_r, p8_r])

    # half plane constraints
    poly_Ab = vert2con(poly_vert)
    poly_Ab = poly_Ab.T
    b = copy.deepcopy(poly_Ab)
    ## normalization the A matrix
    for i in range(0, b.shape[1]):
        a = b[0:3, i]
        poly_Ab[0:3, i] = poly_Ab[0:3, i] / np.linalg.norm(a)
        poly_Ab[3, i] = poly_Ab[3, i] / np.linalg.norm(a)

    poly_vert = poly_vert.T

    return poly_vert, poly_Ab


def box2PolyVertsCons_2D(box_pos = None, box_size = None, orientation_xy = None):
    pass
    x_min = - 0.5 * box_size[0]
    x_max = + 0.5 * box_size[0]
    y_min = - 0.5 * box_size[1]
    y_max = + 0.5 * box_size[1]

    return poly_vert, poly_Ab

def rotz(theta):
    """
    ROTZ Rotation about Z axis
    :param theta: angle for rotation matrix
    :return: rotation array
    """
    ct = math.cos(theta)
    st = math.sin(theta)
    rot = np.array([[ct, -st, 0], [st, ct, 0], [0, 0, 1]])
    # rot = rot.round(15)
    return rot

def box2EllipseVertsCons_3D(box_pos=None, box_size=None, orientation_xy=None, vert_m=None):
    
    t = np.arange(0, 2*np.pi, 2*np.pi/(vert_m*0.5))
    
    x = np.cos(t)*box_size[0]/2
    y = np.sin(t)*box_size[1]/2

    ellipse_x = np.cos(orientation_xy)*x - np.sin(orientation_xy)*y +  box_pos[0]
    ellipse_y = np.sin(orientation_xy)*x + np.cos(orientation_xy)*y +  box_pos[1]
    ellipse_z = np.ones(ellipse_x.shape[0])*box_size[2]

    p_x = np.append(ellipse_x,ellipse_x, axis=0)
    p_y = np.append(ellipse_y,ellipse_y, axis=0)
    p_z = np.append(np.zeros(ellipse_x.shape[0]), ellipse_z, axis=0)
    
    ellipse_vert = np.column_stack((p_x,p_y,p_z))

    # half plane constraints
    ellipse_Ab = vert2con(ellipse_vert)
    ellipse_Ab = ellipse_Ab.T
    b = copy.deepcopy(ellipse_Ab)
    ## normalization the A matrix
    for i in range(0, b.shape[1]):
        a = b[0:3, i]
        ellipse_Ab[0:3, i] = ellipse_Ab[0:3, i] / np.linalg.norm(a)
        ellipse_Ab[3, i] = ellipse_Ab[3, i] / np.linalg.norm(a)


    # # test
    # ax = plt.axes(projection='3d')
    # ax.set_xlim(-35, 35)
    # ax.set_ylim(-35, 35)
    # ax.set_zlim(0, 20)
    # # robot_circle = ax.scatter3D(p_x, p_y, p_z, marker='o', color = 'k',s=50)

    # hull = ConvexHull(ellipse_vert)
    # tri = Poly3DCollection(ellipse_vert[hull.simplices])
    # tri.set_color('b')
    # tri.set_alpha(0.1)
    # poly = ax.add_collection3d(tri)

    # # # plt.plot(ellipse_x,ellipse_y)# 绘制椭圆
    # # plt.axis('auto')
    # # plt.show()
    ellipse_vert = ellipse_vert.T

    return ellipse_vert, ellipse_Ab
