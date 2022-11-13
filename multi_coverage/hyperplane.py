
from box2PolyVertsCons import box2PolyVertsCons_2D, box2PolyVertsCons_3D, box2EllipseVertsCons_3D
import numpy as np
from qpsolvers import solve_qp
from cvxopt import matrix, solvers


def point_point_hyperplane(p1,p2):
    p12 = p2 - p1
    p12_norm = np.linalg.norm(p12)
    assert(p12_norm > 1e-15,'Two points coincide!')
    a = p12 / p12_norm
    b = np.dot(0.5 * np.transpose(a), p1 + p2)
    return a,b

def point_box_shifted_hyperplane(p,pt,size,yaw,vert_m):
    # compute a normalized max-margin shifted separating hyperplane between
    # a point and a box obstacle
    #
    # inputs:
    #   - p: point 1, [dx1], d is the dimension
    #   - pt: box center, [dx1]
    #   - size: box size, [dx1]
    #   - yaw: box yaw, [1]
    #
    # outputs:
    #   - a, b: hyperplane parameters s.t. |a|=1 and a'*p < b, a'*vert(:,i) > b
    #   a: [dx1], b: [1]
    #
    # note: using the SVM maximum margin classifier method


    # convert box to verts
    dim = p.shape[0]
    if dim == 2:
        poly_vert,__ = box2PolyVertsCons_2D(pt,size,yaw)
    else:
        if dim == 3:
            if vert_m == 8:
                poly_vert,__ = box2PolyVertsCons_3D(pt,size,yaw)
            else:
                poly_vert, __ = box2EllipseVertsCons_3D(pt, size, yaw, vert_m)
        else:
            raise Exception('Dimension error!')
    
    # compute the hyperplane
    a,b = point_polytope_shifted_hyperplane(p,poly_vert)
    return a,b



def point_polytope_shifted_hyperplane(p, vert):
    # compute a normalized max-margin shifted separating hyperplane between
    # a point and a polytope
    #
    # inputs:
    #   - p: point 1, [dx1], d is the dimension
    #   - vert: set of vertice to represent the polytope, [dxm]
    #
    # outputs:
    #   - a, b: hyperplane parameters s.t. |a|=1 and a'*p < b, a'*vert(:,i) > b
    #   a: [dx1], b: [1]
    #
    # note: using the SVM maximum margin classifier method

    # dim1, N = p.shape
    dim1 = p.shape[0]
    N = 1
    dim2, M = vert.shape
    assert(dim1 == dim2, 'Dimension error!')
    # maximum margin classifier method
    # H = np.diag(np.array([np.ones((1, dim1)), 0]))
    H = adjConcat(np.diag([1, 1, 1]), np.array([[0]]))
    f = np.zeros((dim1 + 1, 1))
    r1 = np.column_stack((np.transpose(p).reshape(1,p.shape[0]), np.ones((N, 1))))
    r2 = np.column_stack((- np.transpose(vert), - np.ones((M, 1))))
    A = np.row_stack((r1, r2))
    b = - np.ones((M + N, 1))
    # sol = solve_qp(P=H, q=f, G=A, h=b,solver="osqp")
    # sol = solve_qp(P=H, q=f, G=A, h=b)
    # print("QP solution: sol = {}".format(sol))
    # a = sol[0:dim1]
    # b = - sol[dim1]

    H = matrix(H)   
    f = matrix(f)   
    A = matrix(A)   
    b = matrix(b)   
    solution = solvers.qp(H, f, A, b)

    # print(solution['x'])
    sol = solution['x']
    sol = np.array(sol)
    a = sol[0:dim1,0]
    b = - sol[dim1,0]

    # normalization
    a_norm = np.linalg.norm(a)
    a = a / a_norm
    b = b / a_norm
    # shifted to be tight with the polytope
    # min_dis = min(np.dot(a.T, vert) - b)
    # b = b + min_dis
    b = min(np.dot(a.T, vert))
    return a, b

    ## test script
    ## 2D
    # p = [-2; 1];
    # box_center = [0.5; 1];
    # box_size = [2; 1];
    # box_yaw = deg2rad(30);
    # box_vert = box2PolyVertsCons_2D(box_center, box_size, box_yaw);
    # tic
    # [a, b] = point_polytope_shifted_hyperplane(p, box_vert);
    # toc
    # hfig = figure;
    # box on;
    # grid on;
    # axis([-3 3 -3 3]);
    # ax = hfig.CurrentAxes;
    # daspect(ax, [1 1 1]);
    # hold on;
    # plot(p(1), p(2), 'o');
    # hb = plot_poly_vert_2D(ax, box_vert, ...
    #     'FaceColor', [0.4 0.4 0.4], ...
    #     'FaceAlpha', 0.2, ...
    #     'EdgeColor', [0.4 0.4 0.4], ...
    #     'EdgeAlpha', 0.6, ...
    #     'LineWidth', 1.0, ...
    #     'LineStyle', '-.', ...
    #     'SpecularStrength', 0.1, 'AmbientStrength', 0.5);
    # dim = [-3 3; -2 2];
    # hl = plot_line_2D(ax, a, b, dim, ...
    #     'Color', [0.4 0.4 0.4], ...
    #     'LineStyle', '--', ...
    #     'LineWidth', 1.5);

def boundsCon(dim, lb, ub):
    # A_bound: m*dim
    # b_bound: m*1

    if dim == 2:
        A_bound = np.array([[- 1, 0], [0, - 1], [1, 0], [0, 1]])
        b_bound = np.array([[- lb], [ub]])
    else:
        if dim == 3:
            A_bound = np.array([[- 1, 0, 0], [0, - 1, 0], [0, 0, - 1], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
            b_bound = np.hstack((- lb, ub)).reshape(6,1)

    return A_bound, b_bound


def adjConcat(a, b):
    m1, n1 = a.shape
    m2, n2 = b.shape
    left = np.row_stack((a, np.zeros((m2, n1))))
    right = np.row_stack((np.zeros((m1, n2)), b))
    result = np.hstack((left, right))
    return result