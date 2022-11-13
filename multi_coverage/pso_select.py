import numpy as np
    
def pso_select(map = None,p_cur = None,pt_pre = None,boxSize = None): 
# this function find the best solution, based on drone's current positon
# p_cur and the predicted target's position pt_pre
    
    dim = p_cur.shape[1]
    
    ## initialize the particles
    N = 15
    batch = 10
    x = np.zeros((N,dim))
    
    # plimit = map.boundary;  # limit range of position
    plimit = np.array([[- 3,3],[- 3,3],[0,1]])
    r = min(boxSize) / 2
    c1 = 2.0
    c2 = 2.0
    
    # initialize the position of pariticles around spatial point p_pre
    for j in range(0,dim):
        x[:,j] = plimit[j,0] + (plimit[j,1] - plimit[j,0]) * np.random.rand(N,1) + pt_pre[:,j]
    
    ## initialize pariticles
    vel = np.random.rand(N,dim)
    x_best = x
    global_best = np.zeros((1,dim))
    f_x_best = 10000
    f_global_best = 10000
    
    ##  update
    iter = 0
    while iter <= batch:

        # evaluate
        for i in range(0,N):
            w = 0.95 - (0.95 - 0.4) / batch * i
            if (iter != 1):
                # update the velocity
                vel[i,:] = vel[i,:] * w + c1 * rand * (x_best[i,:] - x[i,:]) + c2 * rand * (global_best - x[i,:])
                #  limit velocity
                vel[i,:] = limitRange(vel[i,:],np.array([0,0,0]),1,1)
                # update the position
                x[i,:] = x[i,:] + vel[i,:]
                #limit pisition
                x[i,:] = limitRange(x(i,:),pt_pre,plimit(1,2),plimit(3,2))
            fx = evaluate(map,x(i,:),p_cur,pt_pre,r)
            if f_x_best(i) > fx:
                f_x_best[i] = fx
                x_best[i,:] = x(i,:)
        if f_global_best > np.amin(f_x_best):
            f_global_best,nmin = np.amin(f_x_best)
            global_best = x_best(nmin,:)
        iter = iter + 1

    
    print('The min result:%.2f\n' % (f_global_best))
    print(np.array(['The variable:',num2str(global_best)]))
    print('optimizaiton finished.')
    return global_best

def Dis_min(map=None, p_cal=None, pt_pre=None):
    # this function find the shortest distance from the 3D line to obstacles in
    # a given map; return the coordinate values and minimum distance
    Min = Inf
    res = np.amin(map.xy_res, map.z_res)

    n = np.ceil(norm(p_cal - pt_pre) / res)
    xyz = Divide(p_cal, pt_pre, n)
    for i in np.arange(1, n + 1 + 1).reshape(-1):
        __, dists = findNearestNeighbors(map.ptCloud, xyz(i,:), 1)
        if dists <= Min:
            Min = dists
            coordinate = xyz(i,:)

            return coordinate, Min


def evaluate(map=None, p_cal=None, p_cur=None, pt_pre=None, threshold=None):
    # this function evaluate the cost of solution p_cal based on the map and
    # drone's current position p_cur and target's predicted position pt_pre
    ## weight value
    w1 = 1.0
    w2 = 1.0
    #   w3=1.0;
    ## calculate cost
    cost = 0
    dis_1 = np.abs(norm(p_cal - pt_pre) - threshold)

    __, dis_2 = Dis_min(map, p_cal, pt_pre)

    #  dis_3=norm(p_cal-p_cur);   #calculate the Euclidean distance from p_cal to p_cur

    # the cost include three segments
    cost = cost + dis_1 * w1
    cost = cost + 1 / dis_2 * w2
    #  cost=cost+w3*dis_3;
    return cost

def Divide(p1=None, p2=None, n=None):
    # this function find the n-1 equal diversion points of line p2-p1.
    # out is a (n+1)by3 matrix, which include n-1 equal diversion points and 2 endpoints.
    out = np.zeros((n + 1, 3))
    p_v = p2 - p1

    out[1, :] = p1
    for i in np.arange(1, n - 1 + 1).reshape(-1):
        out[i + 1, 1] = i / n * p_v(1) + p1(1)
        out[i + 1, 2] = i / n * p_v(2) + p1(2)
        out[i + 1, 3] = i / n * p_v(3) + p1(3)

    out[n + 1, :] = p2
    return out

def limitRange(tgt=None, ini=None, ver=None, hor=None):
    # this function limit the value of target based on initial point and distance---dis_max
    if tgt[0] - ini[0] > ver:
        tgt[0] = ini[0] + ver

    if tgt[0] - ini[0] < - ver:
        tgt[0] = ini[0] - ver

    if tgt[1] - ini[1] > ver:
        tgt[1] = ini[1] + ver

    if tgt[1] - ini[1] < - ver:
        tgt[1] = ini[1] - ver

    if tgt[2] - ini[2] > hor:
        tgt[2] = ini[2] + hor

    if tgt[2] - ini[2] < - hor:
        tgt[2] = ini[2] - hor

    return tgt