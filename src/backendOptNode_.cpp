
#include "ros/ros.h"
#include "../include/global_fusion/backendOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>


#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


class GlobalOptNode{

private:
    ros::NodeHandle n;
    GlobalOptimization globalEstimator;
    ros::Publisher pub_global_odometry, pub_global_path, pub_car;


    ros::Publisher  pub_gps_path;

    ros::Publisher  pub_global_befOpt_path;   //把loam转到global下，未优化的pose的轨迹
    ros::Publisher  pub_local_path;    //localPose 轨迹


    ros::Subscriber sub_GPS;
    ros::Subscriber sub_Odom;
    ros::Subscriber sub_Imu;
    /**for debug
    **/
    ros::Publisher pub_lidar_odometry;

    nav_msgs::Path *global_path;
    nav_msgs::Path *gps_path;

    nav_msgs::Path *global_befOpt_path;
    nav_msgs::Path *local_path;
    double last_vio_t;
    std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
    std::queue<sensor_msgs::ImuPtr> imuQueue;
    std::mutex m_buf;
    std::mutex m_imu_mutex;
    //  imu 延迟输入
    int imu_waite_count;
    int imu_waite_threshold;

    bool  Enable_imu;

    double Sync_tolerance;

    // sim gps
    bool  GPS_sim;

public:
    GlobalOptNode():
    n("~")
    {
        global_path         = &globalEstimator.global_path;
        gps_path            = &globalEstimator.gps_path;
        global_befOpt_path  = &globalEstimator.global_befOpt_path;
        local_path          = &globalEstimator.local_path;

        last_vio_t = -1;
        //  imu 延迟输入
        imu_waite_count = 0;
        imu_waite_threshold = 5;

        Enable_imu = false;

    }

    ~GlobalOptNode(){}
    /**
    **/
    void NodeSetup(){

        n.getParam("/enable_imu", Enable_imu);
        n.getParam("/sync_tolerance", Sync_tolerance);
        n.getParam("/gps_sim", GPS_sim);

        sub_GPS = n.subscribe("/gps/fix", 100, &GlobalOptNode::GPS_callback ,this);
        sub_Odom = n.subscribe("/odom_normal", 100, &GlobalOptNode::loam_callback ,this);

        if(Enable_imu)
        {

            sub_Imu = n.subscribe("/imu/data", 1024, &GlobalOptNode::imu_callback,this);

            std::cout<< "setting------------>enable_imu"<<std::endl;
        }
        

        
        pub_global_odometry     = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
        pub_car                 = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);

        pub_lidar_odometry      = n.advertise<nav_msgs::Odometry>("lidar_odometry", 100);

        pub_global_path         = n.advertise<nav_msgs::Path>("global_path", 100);
        pub_gps_path            = n.advertise<nav_msgs::Path>("gps_path", 100);
        pub_global_befOpt_path  = n.advertise<nav_msgs::Path>("global_befOpt_path", 100);

        pub_local_path          = n.advertise<nav_msgs::Path>("local_path", 100);


        // add GPS noise for sim
        globalEstimator.set_sim_gps(GPS_sim);
    }

    /**
    **/
    void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
    {
        //printf("gps_callback! \n");
        m_buf.lock();
        gpsQueue.push(GPS_msg);
        m_buf.unlock();
    }
    /**
    **/
    void imu_callback(const sensor_msgs::ImuPtr& imu_msg)
    {
        m_imu_mutex.lock();
        imuQueue.push(imu_msg);
        m_imu_mutex.unlock();
    }
   
    /**
        TODO: loam_callback
    **/
    void loam_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
    {
        //printf("vio_callback! \n");
        double t = pose_msg->header.stamp.toSec();
        last_vio_t = t;
        Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
        Eigen::Quaterniond vio_q;
        vio_q.w() = pose_msg->pose.pose.orientation.w;
        vio_q.x() = pose_msg->pose.pose.orientation.x;
        vio_q.y() = pose_msg->pose.pose.orientation.y;
        vio_q.z() = pose_msg->pose.pose.orientation.z;
        globalEstimator.inputOdom(t, vio_t, vio_q);

        //**
        //  把 odom--->转到global下的位姿，发布出去
        Eigen::Vector3d lidar_odom_t;
        Eigen:: Quaterniond lidar_odom_q;
        globalEstimator.get_lidar_odom(lidar_odom_t, lidar_odom_q);

        nav_msgs::Odometry lidar_odometry;
        lidar_odometry.header = pose_msg->header;
        lidar_odometry.header.frame_id = "world";     //world
        lidar_odometry.child_frame_id = "world";              //world
        lidar_odometry.pose.pose.position.x = lidar_odom_t.x();
        lidar_odometry.pose.pose.position.y = lidar_odom_t.y();
        lidar_odometry.pose.pose.position.z = lidar_odom_t.z();
        lidar_odometry.pose.pose.orientation.x = lidar_odom_q.x();
        lidar_odometry.pose.pose.orientation.y = lidar_odom_q.y();
        lidar_odometry.pose.pose.orientation.z = lidar_odom_q.z();
        lidar_odometry.pose.pose.orientation.w = lidar_odom_q.w();
        pub_lidar_odometry.publish(lidar_odometry);

        m_buf.lock();
        while(!gpsQueue.empty())
        {
            sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
            double gps_t = GPS_msg->header.stamp.toSec();
            //printf("vio t: %f, gps t: %f \n", t, gps_t);
            // 10ms sync tolerance
            if(gps_t >= t - Sync_tolerance && gps_t <= t + Sync_tolerance)
            {
                //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
                double latitude = GPS_msg->latitude;
                double longitude = GPS_msg->longitude;
                double altitude = GPS_msg->altitude;
                //int numSats = GPS_msg->status.service;
                double pos_accuracy = GPS_msg->position_covariance[0];
                double pos_accuracyLat = 2;
                if(pos_accuracy <= 0)
                    pos_accuracy = 1;
                //printf("receive covariance %lf \n", pos_accuracy);
                //if(GPS_msg->status.status > 8)
                    globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
                gpsQueue.pop();
                break;
            }
            else if(gps_t < t - Sync_tolerance)
                gpsQueue.pop();
            else if(gps_t > t + Sync_tolerance)
                break;
        }
        m_buf.unlock();

        //----------------------------imu-----------------------------------

        m_imu_mutex.lock();
        while(!imuQueue.empty())
        {
            
            sensor_msgs::ImuPtr IMU_msg = imuQueue.front();
            double imu_t = IMU_msg->header.stamp.toSec();
            printf("loam t: %f, imu t: %f \n", t, imu_t);
            // 10ms sync tolerance
            if(imu_t >= t - Sync_tolerance && imu_t <= t + Sync_tolerance)
            {
                Eigen::Quaterniond ImuQ (IMU_msg->orientation.w,IMU_msg->orientation.x,IMU_msg->orientation.y ,IMU_msg->orientation.z);
                
                double roll, pitch, yaw;
                tf::Matrix3x3(tf::Quaternion(ImuQ.x(), ImuQ.y(), ImuQ.z(), ImuQ.w())).getRPY(roll, pitch, yaw);
                std::cout<<"imu 欧拉角roll："<<roll*180 /3.1415f<<",pitch:"<<pitch*180 /3.1415f<<",yaw:"<<yaw*180 /3.1415f<<std::endl;
                if(imu_waite_count<imu_waite_threshold)
                {
                    imu_waite_count++;
                }
                else{

                    globalEstimator.inputIMU(t, ImuQ);   //屏蔽掉
                }
                
                imuQueue.pop();
                break;
            }
            else if(imu_t < t - Sync_tolerance)
                imuQueue.pop();
            else if(imu_t > t + Sync_tolerance)
                break;
        }
        m_imu_mutex.unlock();
        //------------------------------------------------------------------

        Eigen::Vector3d global_t;
        Eigen:: Quaterniond global_q;
        globalEstimator.getGlobalOdom(global_t, global_q);

        nav_msgs::Odometry odometry;
        odometry.header = pose_msg->header;
        odometry.header.frame_id = "world";     //world
        odometry.child_frame_id = "world";              //world
        odometry.pose.pose.position.x = global_t.x();
        odometry.pose.pose.position.y = global_t.y();
        odometry.pose.pose.position.z = global_t.z();
        odometry.pose.pose.orientation.x = global_q.x();
        odometry.pose.pose.orientation.y = global_q.y();
        odometry.pose.pose.orientation.z = global_q.z();
        odometry.pose.pose.orientation.w = global_q.w();
        pub_global_odometry.publish(odometry);
        pub_global_path.publish(*global_path);
        publish_car_model(t, global_t, global_q);

        /**
        just for debug
        **/

        pub_gps_path.publish(*gps_path);
        pub_global_befOpt_path.publish(*global_befOpt_path);

        pub_local_path.publish(*local_path);
    // std::cout<<"欧拉角："<<global_q.matrix().eulerAngles(2,1,0) *180 /3.1415f<<std::endl;
        double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(global_q.x(), global_q.y(), global_q.z(), global_q.w())).getRPY(roll, pitch, yaw);
        std::cout<<"计算欧拉角roll："<<roll*180 /3.1415f<<",pitch:"<<pitch*180 /3.1415f<<",yaw:"<<yaw*180 /3.1415f<<std::endl;
        /////////////////////////////////////

        // write result to file
        std::ofstream foutC("/home/tony-ws1/output/vio_global.csv", ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << pose_msg->header.stamp.toSec() * 1e9 << ",";
        foutC.precision(5);
        foutC << global_t.x() << ","
                << global_t.y() << ","
                << global_t.z() << ","
                << global_q.w() << ","
                << global_q.x() << ","
                << global_q.y() << ","
                << global_q.z() << endl;
        foutC.close();
    }

    void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
    {
        visualization_msgs::MarkerArray markerArray_msg;
        visualization_msgs::Marker car_mesh;
        car_mesh.header.stamp = ros::Time(t);
        car_mesh.header.frame_id = "world";   //world
        car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
        car_mesh.action = visualization_msgs::Marker::ADD;
        car_mesh.id = 0;

        car_mesh.mesh_resource = "./models/car.dae";

        Eigen::Matrix3d rot;
        rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
        
        Eigen::Quaterniond Q;
        Q = q_w_car * rot; 
        car_mesh.pose.position.x    = t_w_car.x();
        car_mesh.pose.position.y    = t_w_car.y();
        car_mesh.pose.position.z    = t_w_car.z();
        car_mesh.pose.orientation.w = Q.w();
        car_mesh.pose.orientation.x = Q.x();
        car_mesh.pose.orientation.y = Q.y();
        car_mesh.pose.orientation.z = Q.z();

        car_mesh.color.a = 1.0;
        car_mesh.color.r = 1.0;
        car_mesh.color.g = 0.0;
        car_mesh.color.b = 0.0;

        float major_scale = 2.0;

        car_mesh.scale.x = major_scale;
        car_mesh.scale.y = major_scale;
        car_mesh.scale.z = major_scale;
        markerArray_msg.markers.push_back(car_mesh);
        pub_car.publish(markerArray_msg);
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "backend_Estimator2");
    ROS_INFO("\033[1;32m---->\033[0m backend_Estimator2 Started.");
    GlobalOptNode   GON;
    GON.NodeSetup();
    
    ros::spin();
    return 0;
}
