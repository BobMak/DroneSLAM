import copy
import os

import cv2
import numpy as np
import open3d as o3d
from open3d.cuda.pybind.geometry import Image
from open3d.cuda.pybind.camera import PinholeCameraIntrinsic

from DPT.util.io import read_pfm
from o3dutils import preprocess_point_cloud, execute_global_registration, refine_registration, full_registration


def calaculte_point_cloud(path, use_rgbd_odometry=False, use_cached=False, intrinsic_path="djitello_intrinsic.json"):
    imgs = os.listdir(os.path.join(path, 'image'))
    imgs.sort()
    # camera intrinsic
    intrinsic = o3d.io.read_pinhole_camera_intrinsic(intrinsic_path)
    imgs = imgs
    # calculating extrinsic matrices for the sequence of frames
    # either using the IMU odometry, which is more prone to drift,
    # or the RGB-D odometry
    extrinsics = []
    def get_imu_extrinsic(i, imu_data):
        # extrinsic matrix
        ang = np.deg2rad((imu_data[i, 1:4]).reshape(3, 1))
        Rx = np.eye(3)
        Rx[1, 1], Rx[2, 2] = np.cos(ang[1]), np.cos(ang[1])
        Rx[1, 2], Rx[2, 1] = -np.sin(ang[1]), np.sin(ang[1])
        Ry = np.eye(3)
        Ry[0, 0], Ry[2, 2] = np.cos(ang[0]), np.cos(ang[0])
        Ry[0, 2], Ry[2, 0] = np.sin(ang[0]), -np.sin(ang[0])
        Rz = np.eye(3)
        Rz[0, 0], Rz[1, 1] = np.cos(ang[2]), np.cos(ang[2])
        Rz[0, 1], Rz[1, 0] = -np.sin(ang[2]), np.sin(ang[2])
        extrinsic = np.zeros((4, 4))
        # rotation
        extrinsic[0:3, 0:3] = Ry @ Rz @ Rx  # np.matmul(np.matmul(Rx, Ry), Rz)
        # translation
        extrinsic[0, 3], extrinsic[1, 3], extrinsic[2, 3] = pos[1, 0], pos[0, 0], pos[2, 0]
        extrinsic[3, 3] = 1
        return extrinsic
    if use_cached:
        extrinsics = np.load(path + "/extrinsics.npy")
    else:
        if use_rgbd_odometry:
            option = o3d.pipelines.odometry.OdometryOption(depth_max=5.0, depth_diff_max=0.1)
            odo_init = np.identity(4)
            odo = np.identity(4)
            print(option)
            color = Image(cv2.imread(os.path.join(path, 'image', imgs[0])))
            depth, scale = read_pfm(os.path.join(path, 'depth', imgs[0][:-3] + "pfm"))
            target_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, Image(scale * depth), convert_rgb_to_intensity=True, depth_trunc=2.5
            )
            extrinsics.append(odo_init)
            for i, img in enumerate(imgs[1:], start=1):
                color = Image(cv2.imread(os.path.join(path, 'image', img)))
                depth, scale = read_pfm(os.path.join(path, 'depth', img[:-3] + "pfm"))
                source_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color, Image(scale*depth), convert_rgb_to_intensity=True, depth_trunc=2.5
                )

                [success_hybrid_term, trans_hybrid_term,
                 info] = o3d.pipelines.odometry.compute_rgbd_odometry(
                    source_rgbd_image, target_rgbd_image, intrinsic, odo_init,
                    o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option)
                if success_hybrid_term:
                    print(f"Using Hybrid RGB-D Odometry {i}/{len(imgs)}")
                    odo = trans_hybrid_term @ odo
                    extrinsics.append(odo)
                    target_rgbd_image = source_rgbd_image
                    continue
                [success_color_term, trans_color_term,
                 info] = o3d.pipelines.odometry.compute_rgbd_odometry(
                    source_rgbd_image, target_rgbd_image, intrinsic, odo_init,
                    o3d.pipelines.odometry.RGBDOdometryJacobianFromColorTerm(), option)
                if success_color_term:

                    print("Using RGB-D Odometry")
                    odo = odo @ trans_hybrid_term
                    extrinsics.append(odo)
                else:
                    print("failed to find odometry from RGB-D image pair")
                    extr = get_imu_extrinsic(i)
                    extrinsics.append(extr)
                source_rgbd_image = target_rgbd_image
            extrinsics = np.array(extrinsics)
            np.save(path + "/extrinsics.npy", extrinsics)
        else:
            extrinsics.append(np.identity(4))
            imu_data = np.genfromtxt(path + "/data.csv", delimiter=",")[1:]
            # initial odometry
            pos = np.zeros((3, 1))
            vel = np.zeros((3, 1))
            acc = np.array(imu_data[0, -3:]).reshape(3, 1) / 1e3
            for i, img in enumerate(imgs, start=1):
                extrinsic = get_imu_extrinsic(i, imu_data)
                extrinsics.append(extrinsic)
                # update the odometry
                dt = (imu_data[i, 0] - imu_data[i - 1, 0]) / 1e9
                vel += acc * dt
                acc = np.array(imu_data[i, -3:]).reshape(3, 1) / 1e3
                pos += vel * dt + 0.5 * acc * dt * dt
    pcds = []
    pcd_final = o3d.geometry.PointCloud()

    for i, img in enumerate(imgs, start=1):
        clrimg = cv2.imread(os.path.join(path, 'image', img))
        clrimg = cv2.cvtColor(clrimg, cv2.COLOR_BGR2RGB)
        color = Image(clrimg)
        depth, scale = read_pfm(os.path.join(path, 'depth', img[:-3] + "pfm"))
        # depth = o3d.io.read_image(os.path.join(path+'-depth', img[:-3] + "png"))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, Image(scale*depth), convert_rgb_to_intensity=False, depth_trunc=2.5)  #  depth_scale=1000, depth_trunc=100.0
        extrinsic = extrinsics[i-1]
        # get a current cloud
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic, extrinsic)
        pcds.append(pcd)
        # if len(pcds)>1:
        #     pcd_down_src, pcd_fpfh_src = preprocess_point_cloud(pcds[-1], voxel_size=0.02)
        #     pcd_down_tgt, pcd_fpfh_tgt = preprocess_point_cloud(pcds[-2], voxel_size=0.02)
        #     # result_ransac = execute_global_registration(pcd_down_src, pcd_down_tgt, pcd_fpfh_src, pcd_fpfh_tgt, voxel_size=0.02)
        #     threshold = 0.02
        #     reg_p2p = o3d.pipelines.registration.registration_icp(
        #         pcd_down_src, pcd_down_tgt, threshold, extrinsic,
        #         o3d.pipelines.registration.TransformationEstimationPointToPlane())
        #
        #     # refine
        #     result = refine_registration(pcd_down_src, pcd_down_tgt, reg_p2p.transformation, voxel_size=0.02)
        #     # pcds[-2].paint_uniform_color([1, 0.706, 0])
        #     # pcds[-1].paint_uniform_color([0, 0.651, 0.929])
        #     # pcds[-2].transform(result_ransac.transformation)
        #     # o3d.visualization.draw_geometries([pcds[-2], pcds[-1]],
        #     #                                   zoom=0.4559,
        #     #                                   front=[0.6452, -0.3036, -0.7011],
        #     #                                   lookat=[1.9892, 2.0208, 1.8945],
        #     #                                   up=[-0.2779, -0.9482, 0.1556])
        #     # pcds[-2].transform(result.transformation)
        #     # result = o3d.pipelines.registration.registration_fgr_based_on_correspondence(
        #     #     pcd_down_src, pcd_down_tgt, result_ransac.correspondence_set,
        #     #     o3d.pipelines.registration.FastGlobalRegistrationOption())
        #     pcds[-1].transform(result.transformation)
        # pcd_final += pcds[-2]

    # registration and pose graph optimization - local and loop detection
    voxel_size = 0.02
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    pcds_down = [pdc.voxel_down_sample(voxel_size=voxel_size) for pdc in pcds]
    for pcd in pcds_down:
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    pose_graph = full_registration(pcds_down, max_correspondence_distance_coarse, max_correspondence_distance_fine)
    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.10,
        reference_node=0)
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    o3d.visualization.draw_geometries(pcds_down)

    # subsample
    # pcds = [pcd.voxel_down_sample(voxel_size=0.05) for pcd in pcds]
    # o3d.visualization.draw_geometries(pcds)
    # o3d.visualization.draw_geometries([pcd_final.voxel_down_sample(voxel_size=0.01)])


if __name__ == "__main__":
    # calaculte_point_cloud("data/43aac46e-560a-4534-8b2a-a97ca337ac49", True, False,)  # longer
    # intrinsic_path='phoneintrinsic.json'
    # calaculte_point_cloud("data/7accc4d7-8d95-4b43-9563-e4d43b53ff29", True, True)  # longer
    # calaculte_point_cloud("data/fb17ee6f-0883-4729-8273-2fca53bdaff6", True, True)  # shorter
    # longer tello
    # calaculte_point_cloud("data/629b5208-3b84-483a-85bf-401f8cdb7138", True, True)  # longer
    # slower tello
    calaculte_point_cloud("data/b8f2da5c-d7d2-4b52-ba95-3e4896064202", True, False)  # longer