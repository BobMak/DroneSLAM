# Drone Active SLAM

## Setup

We are using isl-org/DPT for depth map generation:    
`git clone git@github.com:isl-org/DPT.git`

The project utilizes nmcli for managing wifi connection. Connect to a Tello Drone via Wifi once.
Include a `telloid.txt` file with Tello's SSID in the root directory.

## Running

`python gstrcap.py` - connects to Tello and records video stream along with state to `data/` directory.
On keyboard interrupt, will stop recording and start generating depth images from saved data.

`python orbslam.py` - runs simple point cloud generation based on rgbd data. Currently doesn't utilize
visual odometry for accurate odometry tracking.

## TODO

- [ ] Fix and test extrinsic matrix calculation
- [ ] Implement the Extended Kalman Filter
- [ ] add Kitti dataset format support
- [ ] Compare with ORB-SLAM3
- [ ] Compare with Kimera-VIO