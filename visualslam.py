import os
import struct
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from open3d.cuda.pybind.geometry import Image
from open3d.cuda.pybind.camera import PinholeCameraIntrinsic

# tello camera intrinsics
camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

voxel_size = 0.02
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5


def pairwise_registration(source, target):
    print("Apply point-to-plane ICP")
    # icp_coarse = o3d.pipelines.registration.registration_icp(
    #     source, target, max_correspondence_distance_coarse, np.identity(4),
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane())
    # icp_fine = o3d.pipelines.registration.registration_icp(
    #     source, target, max_correspondence_distance_fine,
    #     icp_coarse.transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane())
    # transformation_icp = icp_fine.transformation
    # information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
    #     source, target, max_correspondence_distance_fine,
    #     icp_fine.transformation)
    threshold = 0.02
    trans_init = np.asarray(
        [[0.862, 0.011, -0.507, 0.5],
            [-0.139, 0.967, -0.215, 0.7],
            [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    transformation_icp = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    return transformation_icp, {}


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id])
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=True))
    return pose_graph


def calaculte_point_cloud(path):
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    imgs = os.listdir(path)
    imgs.sort()
    # camera intrinsic
    intrinsic = PinholeCameraIntrinsic(960, 720, 921.170702, 919.018377, 459.904354, 351.238301)
    # orb_extractor = ORBExtractor()
    imu_data = np.genfromtxt(path + "-data.csv", delimiter=",")[1:]
    # initial odometry
    pos = np.zeros((3, 1))
    vel = np.zeros((3, 1))
    acc = np.array(imu_data[0, -3:]).reshape(3, 1) / 1e2
    pcds = []
    for i, img in enumerate(imgs[1:5], start=1):
        color = Image(cv2.imread(os.path.join(path, img)))
        im = cv2.imread(os.path.join(path+'-depth', img[:-3] + "png"), cv2.IMREAD_ANYDEPTH)
        depth = Image(im)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False, depth_trunc=35)  #  depth_scale=1000, depth_trunc=100.0
        # get a current cloud
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
        pcds.append(pcd)

    # o3d.visualization.draw_geometries(pcds)

    print("Full registration ...")
    pose_graph = full_registration(pcds,
                                   max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine)

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
    o3d.pipelines.registration.global_optimization(
        pose_graph, o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(), option)

    print("Transform points and display")
    for point_id in range(len(pcds)):
        print(pose_graph.nodes[point_id].pose)
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
    o3d.visualization.draw_geometries(pcds)

    print("Make a combined point cloud")
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]
    pcd_combined_down = o3d.geometry.voxel_down_sample(pcd_combined,
                                                       voxel_size=voxel_size)
    o3d.io.write_point_cloud("multiway_registration.pcd", pcd_combined_down)
    o3d.visualization.draw_geometries([pcd_combined_down])


if __name__ == "__main__":
    # calaculte_point_cloud("data/3a489323-7475-4b92-a4b9-58aa36c42ae6")  # longer
    calaculte_point_cloud("data/4b5b3963-a3b2-4a65-a689-242a4ef8cfc9")  # shorter