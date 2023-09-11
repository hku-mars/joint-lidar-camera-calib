# joint-lidar-camera-calib
## 1 Introduction
Joint calibration of intrinsic and extrinsic parameters for LiDAR-camera systems in targetless environments. This work aims to calibrate camera intrinsic and LiDAR-camera extrinsic parameters even without a chessboard, while maintaining comparable accuracy with target-based methods. Our method merely requires several textured planes in the scene. As textured planes are ubiquitous in urban environments, this method enjoys broad usability across diverse calibration scenes. A detailed description of the method can be found in our [paper](https://arxiv.org/abs/2308.12629). The calibration pipeline is summarized in the following figure.
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

If an initial guess of the extrinsic parameters is unavailable, users can recover them using hand-eye-calibration. Sample code (src/hand_eye_calib.cpp) and pose files (sample_data/hand_eye_calib) are provided.

### 4.2 Initialization
As shown in the calibration pipeline, the initilization stage first conducts camera self-calibration and LiDAR pose estimation. 
#### 4.2.1 Camera Self-Calibration
We use the open-source software [COLMAP](https://github.com/colmap/colmap), and we will provide a video detailing how to use it. 
#### 4.2.2 LiDAR Pose Estimation
A slightly modified version of [BALM2](https://github.com/hku-mars/BALM) is provided here. First, estimate each LiDAR pose using incremental point-to-plane registration (input your data path in config/registration.yaml):
```
    roslaunch balm2 registration.launch
```
Next, conduct LiDAR bundle adjustment (input your data path in config/conduct_BA.yaml):
```
    roslaunch balm2 conduct_BA.launch
```

### 4.3 Joint Calibration
### 4.4 Adaptability

## 5 Acknowledgements
In development of this work, we stand on the state-of-the-art works: [COLMAP](https://github.com/colmap/colmap) and [BALM2](https://github.com/hku-mars/BALM).

## 6 License
The source code is released under GPLv2 license.

For commercial use, please contact Dr. Fu Zhang fuzhang@hku.hk.
