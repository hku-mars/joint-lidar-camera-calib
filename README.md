# joint-lidar-camera-calib
## 1 Introduction
Joint calibration of intrinsic and extrinsic parameters for LiDAR-camera systems in targetless environments. This work aims to calibrate camera intrinsic and LiDAR-camera extrinsic parameters even without a chessboard, while maintaining comparable accuracy with target-based methods. Our method merely requires several textured planes in the scene. As textured planes are ubiquitous in urban environments, this method enjoys broad usability across diverse calibration scenes. A detailed description of the method can be found in our [paper](https://arxiv.org/abs/2308.12629). The calibration pipeline is summarized in the following figure.
<div align="center">
<img src="https://github.com/hku-mars/joint-lidar-camera-calib/blob/main/pipeline.png" width="98%" />
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

## 3 Usage
### 3.1 Adaptability
This calibration method works for both solid-state and mechanically spinning LiDARs. In terms of cameras, current version supports the pinhole model with radial and tangential distortion.
### 3.2 Data Collection
### 3.3 File Organization
### 3.4 Initialization
### 3.5 Joint Calibration

## 4 Sample data
Data can be downloaded from this [link](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/llihku_connect_hku_hk/EhBsk9-Nc-dHlssTHgi7L1sBg1fL8PUzG6gy0olccXYT4g?e=BQIBgF).
