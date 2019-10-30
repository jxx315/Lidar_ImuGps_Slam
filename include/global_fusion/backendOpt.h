#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"

#include <opencv-3.3.1-dev/opencv2/core/core.hpp>

using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy_Lat,	double posAccuracy_Lon ,double altitude_Alt);

	void inputIMU(double t ,Eigen::Quaterniond ImuQ);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);


	void get_lidar_odom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);

	void set_sim_gps(const bool sim);
	nav_msgs::Path global_path;

	nav_msgs::Path gps_path;

	nav_msgs::Path global_befOpt_path;
	nav_msgs::Path local_path;

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void optimize();
	void optimize2();
	void updateGlobalPath();
	void updateGPSPath();    ///GPS path
    void update_globalPose_befOpt_MapPath();    // 全局pose，在优化前的轨迹
	void update_localpose_MapPath();
	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
	map<double, vector<double>> GPSPositionMap;

	map<double, vector<double>> IMUOrientationMap;


	map<double, vector<double>> globalPose_befOpt_Map;

	
	bool initGPS;
	bool newGPS;
	bool newIMU;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;


	/**
	 just for debug
	**/
	Eigen::Vector3d lidar_odomP;
	Eigen::Quaterniond lidar_odomQ;
	Eigen::Matrix4d GPSBody_T_world;
	Eigen::Matrix4d Lidar_T_gps;

	Eigen::Quaterniond Q_imu_init_2_W;

	Eigen::Matrix4d T_imu_init_2_W;


	Eigen::Matrix4d lidar_T_W;


	/**
	add gps noise 
	**/
	bool simulation;
	double w_sigma;// 噪声Sigma值
	cv::RNG rng; // OpenCV随机数产生器

};