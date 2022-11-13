import numpy as np
from sklearn.cluster import KMeans
from scipy.optimize import linprog
import matplotlib.pyplot as plt

def cluster_centroid(vpt = None,nSub = None): 
    ptNum = np.zeros((1,nSub))

    sub_vpt = np.empty((1,nSub),dtype=object)

    estimator = KMeans(n_clusters=nSub, init='k-means++', n_init=10, max_iter=300, tol=0.0001, verbose=0, random_state=None,
           copy_x=True, algorithm='auto').fit(vpt)
    label_pred = estimator.labels_
    centroids = estimator.cluster_centers_
    inertia = estimator.inertia_

    for iSub in range(0,nSub):
        ptNum[0,iSub] = len(label_pred == iSub)
        ptIdx = np.where(label_pred == iSub)
        sub_vpt[0,iSub] = vpt[ptIdx]

    return ptNum,centroids,label_pred, sub_vpt


def pixel_centroid(gmap = None,polyhedra = None): 
    dim = gmap.obs_free_pt.shape[1]
    if dim == 3:
        safe_region_pixel = poly_pixel_area_3D(gmap,polyhedra)
    elif dim == 2:
        safe_region_pixel = poly_pixel_area_2D(gmap,polyhedra)
    
    centroid = oCenter(gmap,safe_region_pixel)

    return centroid,safe_region_pixel
    

def poly_pixel_area_3D(gmap=None, polyhedra=None):
    safe_region_pixel = gmap.density_map.reshape((np.array(gmap.density_map).size, 1), order="F")
    bool = in_hull_cvx(gmap.all_pt,polyhedra)
    # plot_in_hull(pt, polyhedra) # 2D data
    safe_region_pixel[bool == 0] = 0
    safe_region_pixel = safe_region_pixel.reshape(np.array([gmap.nx, gmap.ny, gmap.nz]), order="F")

    return safe_region_pixel

def oCenter(gmap=None, pixel_region=None):
    x_range = np.arange(gmap.boundary[0, 0] + gmap.xy_res / 2, gmap.boundary[0, 1] - gmap.xy_res / 2 + gmap.xy_res, gmap.xy_res)
    y_range = np.arange(gmap.boundary[1, 0] + gmap.xy_res / 2, gmap.boundary[1, 1] - gmap.xy_res / 2 + gmap.xy_res, gmap.xy_res)
    z_range = np.arange(gmap.boundary[2, 0] + gmap.z_res / 2, gmap.boundary[2, 1] - gmap.z_res / 2 + gmap.z_res, gmap.z_res)

    sumPixel = np.sum(pixel_region)
    mass_x = np.dot(x_range, np.sum(pixel_region, axis=(1, 2))) / sumPixel
    mass_y = np.dot(np.sum(pixel_region, axis=(0, 2)), y_range.T) / sumPixel
    mass_z = np.dot(z_range, np.sum(pixel_region, axis=(0, 1))) / sumPixel
    centroid = np.row_stack(([mass_x], [mass_y], [mass_z]))
    return centroid

def in_hull(points, x):
    n_points = len(points)
    n_dim = len(x)
    c = np.zeros(n_points)
    A = np.r_[points.T, np.ones((1, n_points))]
    b = np.r_[x, np.ones(1)]
    lp = linprog(c, A_eq=A, b_eq=b)
    return lp.success

def in_hull_cvx(p, hull):
    """
    Test if points in `p` are in `hull`

    `p` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    """
    from scipy.spatial import Delaunay
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p)>=0

def plot_in_hull(p, hull):
    """
    plot relative to `in_hull` for 2d data
    """
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection, LineCollection
    from scipy.spatial import Delaunay
    
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    # plot triangulation
    poly = PolyCollection(hull.points[hull.vertices], facecolors='w', edgecolors='b')
    plt.clf()
    plt.title('in hull')
    plt.gca().add_collection(poly)
    plt.plot(hull.points[:,0], hull.points[:,1], 'o', hold=1)


    # plot the convex hull
    edges = set()
    edge_points = []

    def add_edge(i, j):
        """Add a line between the i-th and j-th points, if not in the list already"""
        if (i, j) in edges or (j, i) in edges:
            # already added
            return
        edges.add( (i, j) )
        edge_points.append(hull.points[ [i, j] ])

    for ia, ib in hull.convex_hull:
        add_edge(ia, ib)

    lines = LineCollection(edge_points, color='g')
    plt.gca().add_collection(lines)
    plt.show()

    # plot tested points `p` - black are inside hull, red outside
    inside = in_hull(p,hull)
    plt.plot(p[ inside,0],p[ inside,1],'.k')
    plt.plot(p[-inside,0],p[-inside,1],'.r')

    
