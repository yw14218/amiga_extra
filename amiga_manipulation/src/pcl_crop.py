import numpy as np
import open3d as o3d
import math

def plane_seg():
    pcd = o3d.io.read_point_cloud("/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/pointclouds/cropped_05.pcd")
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.1,
                                            ransac_n=10,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    outlier_cloud.paint_uniform_color([0, 1, 0])
    print(inlier_cloud)   # 位于方框内的点云点的个数
    print(outlier_cloud)  # 位于方框外的点云点的个数
    o3d.visualization.draw_geometries([outlier_cloud])

def crop_filter(cloud, min_x=0, max_x=math.inf, min_y=0, max_y=math.inf, min_z=1, max_z=math.inf):

    points = np.asarray(cloud.points)

    ind = np.where((points[:, 0] >= min_x) & (points[:, 0] <= max_x) &
                   (points[:, 1] >= min_y) & (points[:, 1] <= max_y) &
                   (points[:, 2] >= min_z) & (points[:, 2] <= max_z))[0]

    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    return inlier_cloud, outlier_cloud

#  获取xyz方向的最大值和最小值
def get_min_max_3d(cloud):
    points = np.asarray(cloud.points)
    min_pt = np.amin(points, axis=0)
    max_pt = np.amax(points, axis=0)
    return min_pt, max_pt

pcd = o3d.io.read_point_cloud("/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/pointclouds/05.pcd")

# [min_xyz, max_xyz] = get_min_max_3d(pcd)
# print('x,y,z方向的最小值分别为：', min_xyz)
# print('x,y,z方向的最大值分别为：', max_xyz)

# in_box_cloud, out_box_cloud = crop_filter(pcd,
#                                           min_x=-math.inf, max_x=math.inf,
#                                           min_y=-math.inf, max_y=math.inf,
#                                           min_z=-math.inf, max_z=1)
# o3d.io.write_point_cloud("out.pcd", in_box_cloud)
# in_box_cloud.paint_uniform_color([1.0, 0, 0])   # 方框内的点渲染成红色
# print(in_box_cloud)   # 位于方框内的点云点的个数
# print(out_box_cloud)  # 位于方框外的点云点的个数

# o3d.visualization.draw_geometries([in_box_cloud, out_box_cloud])
pcd = o3d.io.read_point_cloud("/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/pointclouds/cropped_05.pcd")

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
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
o3d.io.write_point_cloud("seg_ground_05.pcd", outlier_cloud)




