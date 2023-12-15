# Drone Active SLAM

## Setup

We are using isl-org/DPT for depth map generation:
1. clone the repo:`git clone git@github.com:isl-org/DPT.git`   
2. download the nyu funetuned model: `wget https://github.com/intel-isl/DPT/releases/download/1_0/dpt_hybrid_nyu-2ce69ec7.pt`
3. setup and activate the new python environment `conda create -n droneslam python=3.10 && conda activate droneslam`
4. install the requirements: `pip install -r requirements.txt`


The project utilizes nmcli for managing wifi connection. Connect to a Tello Drone via Wifi once.

## Running

### Virtual Pipeline
1. Import [droneSLAM.zip](https://drive.google.com/file/d/1lVXLiruhnDSlGnoww1NBkuen0tFf5uMU/view?usp=sharing) as a project in Unreal Engine 5.3.0.
2. load the NewLevel map and press play. The system will start sharing data on `/droneslam` shared memory buffer.
3. run `python read_unreal_data.py` to read and display the shared memory.

### SLAM Pipeline

1. collect the data   
`python gstrcap.py` - connects to Tello and records video stream along with state to `data/` directory.
On keyboard interrupt, will stop recording and start generating depth images from saved data.
> --type [__drone__|webcam]  
> --telloid [dji tello ssid if drone type]   
> --cam [__/dev/video0__]   

2. estimate the point cloud     
`python slam.py` - runs simple point cloud generation based on rgbd data. Currently doesn't utilize
visual odometry for accurate odometry tracking.
> --path [path to data directory in data/]   
> --mode [__make__|visualize]   
> --use-rgbd-odometry [__True__|False] - use visual odometry for odometry tracking or IMU data for extrinsic matrix calculation   
> --use-cached [__True__|False] - use cached extrinsics, if calculated earlier or recalculate    
> --optimize-pose [__True__|False] - optimize the pose graph
> --intrinsic-path [path to intrinsic matrix file] - path to intrinsic matrix file, if not provided, will use dji tello's intrinsic matrix

## TODO

- [x] Fix and test extrinsic matrix calculation
- [ ] Implement the Extended Kalman Filter
- [ ] add Kitti dataset format support
- [ ] Compare with ORB-SLAM3
- [ ] Compare with Kimera-VIO

## Acknowledgements

- [isl-org/DPT](https://github.com/isl-org/DPT) - Vision Transformers for Dense Prediction