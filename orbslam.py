import os
import re
import struct
from pathlib import Path

import cv2
import numpy as np
# from matplotlib import pyplot as plt
import open3d as o3d
from open3d.cuda.pybind.geometry import Image
from open3d.cuda.pybind.camera import PinholeCameraIntrinsic

# from python_orb_slam3 import ORBExtractor
from DPT.util.io import read_pfm

# tello camera intrinsics
camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])


def calaculte_point_cloud(path, use_rgbd_odometry=False, use_cached=False):
    imgs = os.listdir(os.path.join(path, 'image'))
    imgs.sort()
    # camera intrinsic
    intrinsic = PinholeCameraIntrinsic(960, 720, 921.170702, 919.018377, 459.904354, 351.238301)
    # orb_extractor = ORBExtractor()
    imu_data = np.genfromtxt(path + "/data.csv", delimiter=",")[1:]
    # initial odometry
    pos = np.zeros((3, 1))
    vel = np.zeros((3, 1))
    acc = np.array(imu_data[0, -3:]).reshape(3, 1) / 1e3
    imgs = imgs[40:]
    # calculating extrinsic matrices for the sequence of frames
    # either using the IMU odometry, which is more prone to drift,
    # or the RGB-D odometry
    extrinsics = []
    def get_imu_extrinsic(i):
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
            option = o3d.pipelines.odometry.OdometryOption()
            odo_init = np.identity(4)
            odo = np.identity(4)
            print(option)
            color = Image(cv2.imread(os.path.join(path, 'image', imgs[0])))
            depth, scale = read_pfm(os.path.join(path, 'depth', imgs[0][:-3] + "pfm"))
            source_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, Image(scale * depth),
                                                                        convert_rgb_to_intensity=False, depth_trunc=1.5)
            extrinsics.append(odo_init)
            for i, img in enumerate(imgs[1:], start=1):
                color = Image(cv2.imread(os.path.join(path, 'image', img)))
                depth, scale = read_pfm(os.path.join(path, 'depth', img[:-3] + "pfm"))
                target_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, Image(scale*depth), convert_rgb_to_intensity=False, depth_trunc=50)

                [success_hybrid_term, trans_hybrid_term,
                 info] = o3d.pipelines.odometry.compute_rgbd_odometry(
                    source_rgbd_image, target_rgbd_image, intrinsic, odo_init,
                    o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option)
                if success_hybrid_term:
                    print("Using Hybrid RGB-D Odometry")
                    odo = odo @ trans_hybrid_term
                    extrinsics.append(odo)
                    source_rgbd_image = target_rgbd_image
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
            for i, img in enumerate(imgs[1:], start=1):
                extrinsic = get_imu_extrinsic(i)
                extrinsics.append(extrinsic)

    pcds = []
    for i, img in enumerate(imgs, start=1):
        clrimg = cv2.imread(os.path.join(path, 'image', img))
        clrimg = cv2.cvtColor(clrimg, cv2.COLOR_BGR2RGB)
        color = Image(clrimg)
        depth, scale = read_pfm(os.path.join(path, 'depth', img[:-3] + "pfm"))
        # depth = o3d.io.read_image(os.path.join(path+'-depth', img[:-3] + "png"))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, Image(scale*depth), convert_rgb_to_intensity=False, depth_trunc=1.5)  #  depth_scale=1000, depth_trunc=100.0
        extrinsic = extrinsics[i-1]
        # get a current cloud
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic, extrinsic)
        pcds.append(pcd)
        # update the odometry
        dt = (imu_data[i, 0] - imu_data[i-1, 0]) / 1e9
        vel += acc * dt
        acc = np.array(imu_data[i, -3:]).reshape(3, 1) / 1e3
        pos += vel * dt + 0.5 * acc * dt * dt
    # subsample
    pcds = [pcd.voxel_down_sample(voxel_size=0.01) for pcd in pcds]
    o3d.visualization.draw_geometries(pcds)


if __name__ == "__main__":
    calaculte_point_cloud("data/7accc4d7-8d95-4b43-9563-e4d43b53ff29", True, False)  # longer
    # calaculte_point_cloud("data/fb17ee6f-0883-4729-8273-2fca53bdaff6", True, True)  # shorter