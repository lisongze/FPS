import numpy as np
import open3d as o3d
import math
import time
import sys

def distance_point3d(p0, p1):
    d = (p0[0] - p1[0])**2 + (p0[1] - p1[1])**2 + (p0[2] - p0[2])**2
    return math.sqrt(d)

def furthest_point_sample(points, sample_count):
    points_index = np.arange(points.shape[0], dtype=np.int)
    A = np.array([np.random.choice(points_index)])
    B = np.setdiff1d(points_index, A)
    print(A)
    print(B)
    min_dis_B2A = []
    for i in range(len(B)):
        Pa_index = A[0]
        Pb_index = B[i]
        Pa = points[Pa_index]
        Pb = points[Pb_index]
        dis = distance_point3d(Pb, Pa)
        min_dis_B2A.append(dis)
    min_dis_B2A = np.array(min_dis_B2A)
    print('iter ', len(A), ': ', A)
    while len(A) < sample_count:
        longest_points_in_B_index = np.argmax(min_dis_B2A)
        longest_points_index = B[longest_points_in_B_index]

        # update A and B
        A = np.append(A, longest_points_index)
        B = np.delete(B, longest_points_in_B_index)
        min_dis_B2A = np.delete(min_dis_B2A, longest_points_in_B_index)

        # update min_dis_B2A
        for i in range(len(B)):
            Pa_index = A[-1]
            Pb_index = B[i]
            Pa = points[Pa_index]
            Pb = points[Pb_index]
            dis = distance_point3d(Pb, Pa)
            min_dis_B2A[i] = min(dis, min_dis_B2A[i])
        
        print('iter ', len(A), ': ', A)

    return A

class RenderClass(object):
    def __init__(self, pcd, points, sampled_index):
        self.pcd = pcd
        self.points = points
        self.sampled_index = sampled_index
        self.current_render_index = 0
    
    def vis_callback(self, vis):
        if self.current_render_index < len(self.sampled_index):
            mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
            mesh_sphere.paint_uniform_color([0.8, 0.1, 0.2])
            mesh_sphere.translate(self.points[self.sampled_index[self.current_render_index]])
            self.current_render_index = self.current_render_index + 1
            time.sleep(0.1)
            vis.add_geometry(mesh_sphere, False)
        else:
            vis.clear_geometries()
            vis.add_geometry(self.pcd, False)
            self.current_render_index = 0



if __name__=='__main__':
    ply_file = sys.argv[1]
    pcd = o3d.io.read_point_cloud(ply_file)
    pcd.paint_uniform_color([0.5,0.5,0.5])
    pcd_array = np.asarray(pcd.points)
    sample_count = 128
    sampled_points_index = furthest_point_sample(pcd_array, sample_count)

    content = [pcd]

    rc = RenderClass(pcd, pcd_array, sampled_points_index)

    o3d.visualization.draw_geometries_with_animation_callback(content, rc.vis_callback)

    #for i in sampled_points_index:
    #    point = pcd_array[i]
    #    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
    #    mesh_sphere.paint_uniform_color([0.8, 0.1, 0.2])
    #    mesh_sphere.translate(point)
    #    content.append(mesh_sphere)
    #o3d.visualization.draw_geometries(content)