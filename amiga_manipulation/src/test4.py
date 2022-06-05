import numpy as np
import open3d as o3d
import math

pcd = o3d.io.read_point_cloud("/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/pointclouds/05.pcd")
#o3d.visualization.draw_geometries([pcd])
points = np.asarray(pcd.points)
print(points.shape)

def crop_filter(cloud, min_x=0, max_x=math.inf, min_y=0, max_y=math.inf, min_z=1, max_z=math.inf):

    points = np.asarray(cloud.points)
    ind = np.where((points[:, 0] >= min_x) & (points[:, 0] <= max_x) &
                   (points[:, 1] >= min_y) & (points[:, 1] <= max_y) &
                   (points[:, 2] >= min_z) & (points[:, 2] <= max_z))[0]

    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    return inlier_cloud, outlier_cloud


import time
start = time.time()
plane_model, inliers = pcd.segment_plane(distance_threshold=0.005,
                                        ransac_n=3,
                                        num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
#outlier_cloud.paint_uniform_color([0, 1, 0])
print(inlier_cloud)   # 位于方框内的点云点的个数
print(outlier_cloud)  # 位于方框外的点云点的个数
print(time.time()-start)

o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
o3d.io.write_point_cloud("seg_ground_05.pcd", outlier_cloud)
