import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull

class EnvPlot3D:

    def __init__(self, ifShowAnimation=1, ifShowHistoryTra=0, ifShowSafeRegion=1, save_ani=0, ws=None):
        # visualization setup
        self.ifShowHistoryTra = ifShowHistoryTra
        self.ifShowSafeRegion = ifShowSafeRegion
        self.ifShowVelocity = 0
        self.ifShowAnimation = ifShowAnimation
        self.saveAnimation = save_ani
        self.saveLog = 0

        self.save_ani = save_ani
        self.xlimit = ws[0,:]
        self.ylimit = ws[1,:]
        self.zlimit = ws[2,:]

    # def init_environment(self,pos=np.zeros((3,1)),robot_marker='o',robot_color='r',vert=np.ones((3,1)),obs_color='k'):
    def init_environment(self):

        # fig = plt.figure()
        ax = plt.axes(projection='3d')
        
        # ax.set_aspect('equal')
        ax.set_xlim(self.xlimit[0], self.xlimit[1])
        ax.set_ylim(self.ylimit[0], self.ylimit[1])
        ax.set_zlim(self.zlimit[0], self.zlimit[1])
        
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_ylabel("z [m]")
        ax.view_init(elev=90., azim=0)

        return ax
            
            # self.plot_robot(pos,robot_marker,robot_color)
            # self.plot_polyhedron(vert,obs_color)



    def plot_polyhedron(self,ax,vert = np.ones((8,3)),poly_color = 'k',poly_alpha = 0.1):
        # for vert, color in zip([A, B, C], ['r', 'g', 'b']):
        hull = ConvexHull(vert)
        # draw the polygons of the convex hull
        #         
        tri = Poly3DCollection(vert[hull.simplices])
        tri.set_color(poly_color)
        tri.set_alpha(poly_alpha)
        poly = ax.add_collection3d(tri)

        # draw the vertices
        # ax.scatter(vert[:, 0], vert[:, 1], vert[:, 2], marker='o', color='k')
        # plt.show()
        return poly

    # def plot_robot(self, robot_color = 'g', goal_color='r', show_lidar=True, show_goal=True, show_text=False, show_traj=False, traj_type='-g', fontsize=10):
    def plot_robot(self,ax,pos=np.zeros((3,1)),robot_marker='o',robot_color='r'):
        # x = pos[0, 0]
        # y = pos[1, 0]
        # z = pos[2, 0]
        x = pos[0]
        y = pos[1]
        z = pos[2]
        # goal_x = self.goal[0, 0]
        # goal_y = self.goal[1, 0]
        # goal_z = self.goal[2, 0]
        robot_circle = ax.scatter3D(x, y, z, marker=robot_marker, color = robot_color,s=50)
        # plt.pause(0.1)
        return robot_circle


        # robot_circle = mpl.patches.Circle(xy=(x, y), radius = self.radius, color = robot_color)
        # robot_circle.set_zorder(3)

        # ax.add_patch(robot_circle)
        # if show_text: ax.text(x - 0.5, y, 'r'+ str(self.id), fontsize = fontsize, color = 'r')
        # self.plot_patch_list.append(robot_circle)

        # if show_goal:
        #     goal_circle = mpl.patches.Circle(xy=(goal_x, goal_y), radius = self.radius, color=goal_color, alpha=0.5)
        #     goal_circle.set_zorder(1)
        
        #     ax.add_patch(goal_circle)
        #     if show_text: ax.text(goal_x + 0.3, goal_y, 'g'+ str(self.id), fontsize = fontsize, color = 'k')
        #     self.plot_patch_list.append(goal_circle)



    # def plot_polytope(self, obs_poly_color='k'): 

    #     obs_poly = mpl.patches.Polygon(xy=self.vertex.T, closed=True, color=obs_poly_color)
    #     obs_poly.set_zorder(2)
    #     ax.add_patch(obs_poly)
    #     self.plot_patch_list.append(obs_poly)

    # def clear_components(self, mode='all', **kwargs):
    #     if mode == 'dynamic':
    #         self.env_robot.plot_clear(ax)
    #         [env_obs.plot_clear() for env_obs in self.env_obstacle_list if env_obs.dynamic]
    #         [line.pop(0).remove() for line in self.dyna_line_list]

    #         self.dyna_line_list = []
            
    #     elif mode == 'static':
    #         pass
        
    #     elif mode == 'all':
    #         plt.cla()

    # def draw_trajectory(self, traj, style='g-', label='trajectory', show_direction=False, refresh=False, **kwargs):
    #     # traj: a list of points
    #     if isinstance(traj, list):
    #         path_x_list = [p[0, 0] for p in traj]
    #         path_y_list = [p[1, 0] for p in traj]
    #     elif isinstance(traj, np.ndarray):
    #         # raw*column: points * num
    #         path_x_list = [p[0] for p in traj.T]
    #         path_y_list = [p[1] for p in traj.T]

    #     line = ax.plot(path_x_list, path_y_list, style, label=label, **kwargs)

    #     if show_direction:
    #         if isinstance(traj, list):
    #             u_list = [cos(p[2, 0]) for p in traj]
    #             y_list = [sin(p[2, 0]) for p in traj]
    #         elif isinstance(traj, np.ndarray):
    #             u_list = [cos(p[2]) for p in traj.T]
    #             y_list = [sin(p[2]) for p in traj.T]

    #         ax.quiver(path_x_list, path_y_list, u_list, y_list)

    #     if refresh:
    #         self.dyna_line_list.append(line)

    # def show(self, save_fig=False, fig_name='fig.png', **kwargs):
    #     if self.plot:
    #         self.draw_components(ax, mode='dynamic', **kwargs)
            
    #         if save_fig: self.fig.savefig(fig_name)

    #         logging.info('Program Done')
    #         plt.show()

    # def end(self, ani_name='animation', save_fig=False, fig_name='fig.png', show=True, **kwargs):
        
    #     if self.save_ani: self.save_animate(ani_name, **kwargs)
            
    #     if self.plot:
    #         self.draw_components(ax, mode='dynamic', **kwargs)

    #         if save_fig: self.fig.savefig(fig_name, **kwargs)

    #         if show: plt.show()

    # def save_gif_figure(self, save_figure_format='png', **kwargs):

    #     if not self.image_path.exists(): self.image_path.mkdir()

    #     order = str(self.count).zfill(3)
    #     plt.savefig(str(self.image_path)+'/'+order+'.'+save_figure_format, format=save_figure_format, **kwargs)

    # def save_animate(self, ani_name='animated', keep_len=30, rm_fig_path=True, **kwargs):
        
    #     if not self.ani_path.exists(): self.ani_path.mkdir()
            
    #     images = list(self.image_path.glob('*.png'))
    #     images.sort()
    #     image_list = []
    #     for i, file_name in enumerate(images):

    #         if i == 0: continue

    #         image_list.append(imageio.imread(file_name))
    #         if i == len(images) - 1:
    #             for j in range(keep_len):
    #                 image_list.append(imageio.imread(file_name))

    #     imageio.mimsave(str(self.ani_path)+'/'+ ani_name+'.gif', image_list)
    #     print('Create animation successfully')

    #     if rm_fig_path: shutil.rmtree(self.image_path)

            



    


            