# Stereo SLAM

This work is based on the implementation of [Visual SLAM](https://github.com/gaoxiang12/slambook-en) with ROS2 Humble.
The code is modified from [chapter 12](https://github.com/gaoxiang12/slambook2/tree/master/ch13) of the book.

## 1. Demo Highlights
Watch our demo at [Video Link](https://youtu.be/Z5XTmDap_Pk)
<p align='center'>
  <a href="https://youtu.be/Z5XTmDap_Pk">
    <img width="65%" src="./img/visual_slam.gif"/>
  </a>
</p>

## 2. Prerequisites
### 2.1 **Ubuntu** and **ROS2**
Ubuntu 64-bit 22.04.

ROS2 Humble. [ROS Humble Installation](https://docs.ros.org/en/humble/Installation.html).

### 2.2. **Sophus**
Follow [Sophus Installation](https://github.com/strasdat/Sophus).

### 2.3. **g2o**
Follow [g2o Installation](https://github.com/RainerKuemmerle/g2o).

### 2.4. **GeographicLib**
This library is not necessary. We use the lib only for showing the GPS path. You may remove the GPS related code if it is not needed.
Follow [Geographic Installation](https://geographiclib.sourceforge.io/).

## 3. Build
### 3.1 Clone repository:
```
cd ~/colcon_ws/src
git clone https://github.com/Chris7462/stereo_slam.git
cd ..
colcon build
source ~/colcon_ws/install/setup.bash
```
### 3.2 Download test rosbag
Download [Kitti raw drive 0014](https://www.cvlibs.net/datasets/kitti/raw_data.php). *You have to convert the raw data into ros2 bag*.

### 3.3 Launch ROS
```
ros2 launch stereo_slam stereo_slam_launch.py
```

## 4.Acknowledgements
Thanks for [Visual SLAM](https://github.com/gaoxiang12/slambook-en) book.
