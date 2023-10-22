# joint-lidar-camera-calib
## 1 Introduction
Joint intrinsic and extrinsic LiDAR-camera calibration in targetless environments. This work aims to calibrate camera intrinsic and LiDAR-camera extrinsic parameters even without a chessboard, while maintaining comparable accuracy with target-based methods. Our method merely requires several textured planes in the scene. As textured planes are ubiquitous in urban environments, this method enjoys broad usability across diverse calibration scenes. A detailed description of the method can be found in our [paper](https://arxiv.org/abs/2308.12629). The calibration pipeline is summarized in the following figure.
<div align="center">
<img src="https://github.com/hku-mars/joint-lidar-camera-calib/blob/main/pipeline.png" width="70%" />
</div>

## 2 Installation
### 2.1 Prerequisites
Ubuntu  
ROS  
OpenCV  
PCL  
Eigen  
Ceres Solver  

### 2.2 Build
Clone the repository and catkin_make:

```
cd ~/$A_ROS_DIR$/src
git clone https://github.com/hku-mars/joint-lidar-camera-calib.git
cd ..
catkin_make
source devel/setup.bash
```

## 3 Sample data
Data can be downloaded from this [link](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/llihku_connect_hku_hk/EhBsk9-Nc-dHlssTHgi7L1sBg1fL8PUzG6gy0olccXYT4g?e=BQIBgF).

## 4 Usage
This calibration method works for both solid-state and mechanically spinning LiDARs. In terms of cameras, current version supports the pinhole model with radical and tangential distortion.

### 4.1 Data Collection
The ideal calibration scene usually consists of multiple textured planes, as shown in the following figure. In urban environments, such scenes are ubiquitous. However, please note that at least three planes with non-coplanar normal vectors are needed. Otherwise, the scene leads to degeneration of point-to-plane registration and thus compromises the extrinsic calibration accuracy.
<div align="center">
<img src="https://github.com/hku-mars/joint-lidar-camera-calib/blob/main/0.png" width="50%" />
</div>
In general, users roughly know the extrinsic parameters of the sensor suites. Given initial extrinsic parameters (initial rotation error < 5 degrees, initial tranlation error < 0.5 m), 6~8 frames of data are typically enough to calibrate the parameters. We suggest that, when recording each frame, the sensor suite is kept static to avoid point cloud distortion and sensor synchronization problems. The user changes the yaw angle (about 5 degrees) and x-y translation (about 10 cm) of the sensor suite a little bit when recording a new frame. Note that pure translation (no rotation) should be avoided because this leads to the degeneration of camera self-calibration. Continuous motion is also accepted if the above-mentioned problems can be tackled.  

If an initial guess of the extrinsic parameters is unavailable, users can recover them using hand-eye-calibration. Sample code (*src/hand_eye_calib.cpp*) and pose files (*sample_data/hand_eye_calib*) are provided.

### 4.2 Initialization
As shown in the calibration pipeline, the Initilization stage first conducts **Camera Self-Calibration** and **LiDAR Pose Estimation**. 
#### 4.2.1 Camera Self-Calibration
We use the open-source software [COLMAP](https://github.com/colmap/colmap), and we have a video detailing how to use it. It can be accessed on [Youtube](https://youtu.be/JEqmdhYmR4A) and [OneDrive](https://connecthkuhk-my.sharepoint.com/:v:/g/personal/llihku_connect_hku_hk/EUoPgpWxtL1MucLGLMYu25sBvH8z35W7Fn4t9CqYcu9wdw?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0RpcmVjdCJ9fQ&e=YJLU14).
#### 4.2.2 LiDAR Pose Estimation
A slightly modified version of [BALM2](https://github.com/hku-mars/BALM) is provided here. First, estimate each LiDAR pose using incremental point-to-plane registration (input your data path in *config/registration.yaml*):
```
roslaunch balm2 registration.launch
```
Next, conduct LiDAR bundle adjustment (input your data path in *config/conduct_BA.yaml*):
```
roslaunch balm2 conduct_BA.launch
```

### 4.3 Joint Calibration
Organize your data folder as follows:
```
.
├── clouds
│   ├── 0.pcd
│   ├── ...
│   └── x.pcd
├── config
│   └── config.yaml
├── images
│   ├── 0.png
│   ├── ...
│   └── x.png
├── LiDAR_pose
│   └── lidar_poses_BA.txt
├── result
└── SfM
    ├── cameras.txt
    ├── images.txt
    └── points3D.txt
```
Then conduct **Joint Optimization** (input your data path in *launch/calib.launch*):
```
roslaunch joint_lidar_camera_calib calib.launch
```
Note that the step **Refinement of Visual Scale and Extrinsic Parameters** in the Initilization stage is also executed here.

### 4.4 Adaptability
#### 4.4.1 Extrinsic Calibration Only
If you are very confident in your intrinsic parameters (e.g. you calibrate them using standard target-based method) and only want to optimize extrinsic parameters, set *keep_intrinsic_fixed* to *true* in *config/config.yaml*.
#### 4.4.2 Other Pinhole Models
For the pinhole model with/without distortions, there are multiple combinations of camera intrinsic parameters. For instance, (*fx = fy, cx, cy*), (*fx = fy, cx, cy, k1, k2*), (*fx, fy, cx, cy, k1, k2, p1, p2*), and so on. Current version of this repository provides an implementation of (*fx = fy, cx, cy, k1, k2*), a commonly used group of parameters. However, when users want to use other combinations, they should adapt the corresponding functions (all the functions that work with camera projection) in *include/calib.hpp* to the specific intrinsic parameters.
#### 4.4.3 Fisheye/Panoramic Camera
We have not tested the performance on fisheye/panoramic cameras, so we are not sure about the calibration accuracy on them. Adaptation of code is also required. Users should adapt the corresponding functions (all the functions that work with camera projection) in *include/calib.hpp* to the specific intrinsic parameters.

## 5 Debug
If you get unexpected calibration result on your own dataset, here are some tips for locating the problem(s).  
(1) Visualize the accumulated point cloud after LiDAR BA. After conducting LiDAR BA (see Section 4.2.2), you might want to check the accuracy of LiDAR poses. While it is challenging to quantitatively evaluate the pose accuracy from poses stored in *LiDAR_pose/lidar_poses_BA.txt*, you may qualitatively evaluate it through the accumulated point cloud. The code transforms point cloud of every individual frame into the first LiDAR frame, which is saved as *clouds/cloud_all.pcd*. Visualize it in CloudCompare/PCL Viewer and check point cloud quality, e.g. whether the roads/walls are flat and thin enough.  
(2) Make sure that the extrinsic parameters you provide in *config/config.yaml* are those which transform a point from LiDAR frame to camera frame. They should not be extrinsics which transform a point from camera frame to LiDAR frame or represent any other transformation.  
(3) Make sure you have a good initial guess of extrinsic parameters. Although the initial parameters should, of course, be different from the ground-true values, they should not deviate too much from ground-truth. Rotation error of 5~10 degrees and translation error less than 0.5m are generally acceptable. A good way to check the extrinsic parameters is to colorize point cloud using initial extrinsic and intrisic parameters and visualize it in CloudCompare/PCL Viewer. We provide an implementation of colorization, i.e. function *void Calib::colorize_point_cloud* in *calib.hpp*.  
(4) Check if the refinement module (Section 4.3) works. Visualize LiDAR point cloud (*result/LiDAR_cloud.pcd*) and visual points (*result/SfM_cloud_before_opt.pcd*) in CloudCompare/PCL Viewer. They should align well, as shown in the following figure, where white points are LiDAR point cloud and colored points are visual points.
<div align="center">
<img src="https://github.com/hku-mars/joint-lidar-camera-calib/blob/main/align_refined.png" width="60%" />
</div>

## 6 Acknowledgements
In development of this work, we stand on the state-of-the-art works: [COLMAP](https://github.com/colmap/colmap) and [BALM2](https://github.com/hku-mars/BALM).

## 7 License
The source code is released under GPLv2 license.

For commercial use, please contact Dr. Fu Zhang fuzhang@hku.hk.
