import os

import cv2
import numpy as np
# from matplotlib import pyplot as plt
import open3d as o3d
from open3d.cuda.pybind.geometry import Image
from open3d.cuda.pybind.camera import PinholeCameraIntrinsic

# from python_orb_slam3 import ORBExtractor
# from DPT.util.io import read_pfm

# tello camera intrinsics
camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

def calaculte_point_cloud(path):
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
    for i, img in enumerate(imgs[1:], start=1):
        # depth, scale = read_pfm(os.path.join(path+'-depth', img[:-3] + "pfm"))
        # depth = depth * scale
        color = Image(cv2.imread(os.path.join(path, img)))
        depth = Image(cv2.imread(os.path.join(path+'-depth', img[:-3] + "png"), cv2.IMREAD_ANYDEPTH))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False)
        # extrinsic matrix
        ang = np.deg2rad(np.array(imu_data[i, 1:4]).reshape(3, 1))
        Rx = np.eye(3)
        Rx[[[1,1], [2,2]]] = np.cos(ang[0])
        Rx[[[1,2], [2,1]]] = np.sin(ang[0])
        Rx[1,2] *= -1
        Ry = np.eye(3)
        Ry[[[0,0], [2,2]]] = np.cos(ang[1])
        Ry[[[0,2], [2,0]]] = np.sin(ang[1])
        Ry[2,0] *= -1
        Rz = np.eye(3)
        Rz[[[0,0], [1,1]]] = np.cos(ang[2])
        Rz[[[0,1], [1,0]]] = np.sin(ang[2])
        Rz[0,1] *= -1
        # Txyz = np.eye(4)
        extrinsic = np.zeros((4, 4))
        extrinsic[:3, 3] = pos[:, 0]
        extrinsic[0:3, 0:3] = np.matmul(np.matmul(Rx, Ry), Rz)
        extrinsic[3, 3] = 1
        # extrinsic = np.matmul(np.matmul(Rx, Ry), Rz) + Txyz
        # extrinsic = np.matmul(np.matmul(Rx, Ry), Rz) + Txyz
        # get a current cloud
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic, extrinsic)
        pcds.append(pcd)
        # update the odometry
        dt = (imu_data[i, 0] - imu_data[i-1, 0]) / 1e9
        vel += acc * dt
        acc = np.array(imu_data[i, -3:]).reshape(3, 1) / 1e2
        pos += vel * dt
    o3d.visualization.draw_geometries(pcds)


if __name__ == "__main__":
    calaculte_point_cloud("data/3a489323-7475-4b92-a4b9-58aa36c42ae6")