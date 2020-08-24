# Lidar_ImuGps_Slam
slam with lidar  imu/gps   


Lidar_ImuGps_Slam is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure. This code is modified from LOAM and vins_fusion(https://github.com/HKUST-Aerial-Robotics/VINS-Fusion).


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).


## 2. Build Lidar_ImuGps_Slam
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone git@github.com:jxx315/Lidar_ImuGps_Slam.git
    cd ../
    catkin_make -j8
    source ~/catkin_ws/devel/setup.bash
```


## 3. KITTI Example (Velodyne HDL-64)
Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER and set the `dataset_folder` and `sequence_number` parameters in `kitti_helper.launch` file. Note you also convert KITTI dataset to bag file for easy use by setting proper parameters in `kitti_helper.launch`. 

```
    roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch
    roslaunch aloam_velodyne kitti_helper.launch
```
<img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti_gif.gif" width = 720 height = 351 />


## 6.Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) ,VINS_fustion(https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git)and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).




