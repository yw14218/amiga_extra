import open3d as o3d
from open3d import *
import numpy as np
import matplotlib.pyplot as plt
import math

# def display_inlier_outlier(cloud, ind):
#     inlier_cloud = cloud.select_down_sample(ind)
#     outlier_cloud = cloud.select_down_sample(ind, invert=True)

#     print("Showing outliers (red) and inliers (gray): ")
#     outlier_cloud.paint_uniform_color([1, 0, 0])
#     inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
#     o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    
if __name__ == "__main__":
    demo_crop_data = o3d.data.DemoCropPointCloud()
    pcd = o3d.io.read_point_cloud(demo_crop_data.point_cloud_path)
    o3d.visualization.draw_geometries([pcd])
    print(demo_crop_data.cropped_json_path)
    vol = o3d.visualization.read_selection_polygon_volume(demo_crop_data.cropped_json_path)
    print(vol)
    # chair = vol.crop_point_cloud(pcd)
    # o3d.visualization.draw_geometries([chair],
    #                                 zoom=0.7,
    #                                 front=[0.5439, -0.2333, -0.8060],
    #                                 lookat=[2.4615, 2.1331, 1.338],
    #                                 up=[-0.1781, -0.9708, 0.1608])
    # pcd = o3d.io.read_point_cloud("/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/pointclouds/05.pcd")
    # pcl_np = np.asarray(pcd.points)

    # o3d.visualization.draw_geometries([cropped])

    # print("Downsample the point cloud with a voxel of 0.02")
    # voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    # o3d.visualization.draw_geometries([voxel_down_pcd])
    # print("Every 5th points are selected")
    # uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
    # o3d.visualization.draw_geometries([uni_down_pcd])

    # print("Statistical oulier removal")
    # cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,
    #                                                     std_ratio=2.0)
    # display_inlier_outlier(voxel_down_pcd, ind)

    # print("Radius oulier removal")
    # cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.05)
    # display_inlier_outlier(voxel_down_pcd, ind)

    #pcl_np = np.asarray(pcd.points)

    # center = np.array([1.586, -8.436, -0.242])
    # radius = 0.5

    # distances = np.linalg.norm(pcl_np - center, axis=1)
    # pcd.points = o3d.utility.Vector3dVector(pcl_np[distances <= radius])

    # Write point cloud out
    #open3d.io.write_point_cloud("out.ply", pcd1
    # o3d.visualization.draw(pcd)
    