#include "../include/global_fusion/backendOpt.h"
#include "../include/global_fusion/Factors.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


///
GlobalOptimization::GlobalOptimization()
{
	initGPS = false;
    newGPS = false;
	WGPS_T_WVIO = Eigen::Matrix4d::Identity();
    threadOpt = std::thread(&GlobalOptimization::optimize, this);

    //////////////////
    Lidar_T_gps << 0.0960,    0.0464,    0.9943,    0.1994,
                   0.9947,    0.0323,   -0.0975,    1.3360,
                  -0.0366,    0.9984,   -0.0431,    1.7000,
                    0,         0,         0,        1.0000;

  Q_imu_init_2_W.x() =  -0.0199451436395398;   //[-0.0199451436395398	-0.00917132445249710	-0.999753309870762	0.00337601176555534]
  Q_imu_init_2_W.y() = 	-0.00917132445249710;
  Q_imu_init_2_W.z() =  -0.999753309870762;
  Q_imu_init_2_W.w() =  0.00337601176555534;


  Eigen::Matrix3d tempR =Q_imu_init_2_W.matrix();
  Eigen::Vector3d tempt(0,0,0);
  T_imu_init_2_W = Eigen::Matrix4d::Identity(); ;
  T_imu_init_2_W.block<3, 3>(0, 0) = tempR;
  T_imu_init_2_W.block<3, 1>(0, 3) = tempt;


  lidar_T_W = T_imu_init_2_W *Lidar_T_gps;


  //WGPS_T_WVIO = lidar_T_W;

  // gps noise
//   simulation = true;
  //w_sigma = 1.0f;

}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}

void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();

    //**********************************************************************
    Eigen::Matrix4d OdomPQ= Eigen::Matrix4d::Identity(); 
    OdomPQ.block<3, 3>(0, 0) = OdomQ.toRotationMatrix();
    OdomPQ.block<3, 1>(0, 3) = OdomP;
    // 
    printf("raw_odom: t: %f x: %f y: %f z:%f \n", t, OdomP.x(), OdomP.y(), OdomP.z());
    Eigen::Matrix4d Rtlb2la = Eigen::Matrix4d::Identity(); 
    //    Rtlb2la<<-1, 0, 0,  0 ,     //之前的测试ok.
    //              0, 0, 1,  0 ,
    //              0 ,1, 0,  0 ,
    //              0 ,0, 0,  1 ;
          Rtlb2la<< 1, 0, 0,  0 ,    //测试也ok
                    0, 1, 0,  0 ,
                    0 ,0, 1,  0 ,
                    0 ,0, 0,  1 ;
    
    Eigen::Matrix4d  loamBody_2_ENUBody =    Rtlb2la *OdomPQ;

    Eigen::Quaterniond  OdomQ_ENUBody(loamBody_2_ENUBody.block<3, 3>(0, 0) );
    Eigen::Vector3d OdomP_ENUBody(loamBody_2_ENUBody.block<3, 1>(0, 3));

    //***********************************************************************

    // vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    // 					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    vector<double> localPose{OdomP_ENUBody.x(), OdomP_ENUBody.y(), OdomP_ENUBody.z(), 
    					     OdomQ_ENUBody.w(), OdomQ_ENUBody.x(), OdomQ_ENUBody.y(), OdomQ_ENUBody.z()};
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(OdomQ.x(), OdomQ.y(), OdomQ.z(), OdomQ.w())).getRPY(roll, pitch, yaw);
    std::cout<<"原生loam 欧拉角roll："<<roll*180 /3.1415f<<",pitch:"<<pitch*180 /3.1415f<<",yaw:"<<yaw*180 /3.1415f<<std::endl;

    localPoseMap[t] = localPose;

    //**********************************************************************

    //   double roll, pitch, yaw;
    //  tf::Matrix3x3(tf::Quaternion(OdomQ.x(), OdomQ.y(), OdomQ.z(), OdomQ.w())).getRPY(roll, pitch, yaw,1);
    // std::cout<<"欧拉角roll-->"<<roll*180 /3.1415f<<",pitch-->"<<pitch*180 /3.1415f<<",yaw-->"<<yaw*180 /3.1415f<<std::endl;

    // //  just for test
    //  Eigen::Quaterniond temp = OdomQ;
    // Eigen::Matrix4d OdomPQ= Eigen::Matrix4d::Identity(); 
    // // 
    // Eigen::Matrix4d Rtlb2la = Eigen::Matrix4d::Identity(); 
    //   Rtlb2la<<0, -1, 0 , 0 ,
    //            1, 0 ,0 , 0 ,
    //            0 ,0,  -1,  0 ,
    //            0 ,0,  0 ,  1 ;
             
    
    //  OdomPQ.block<3, 3>(0, 0) = OdomQ.toRotationMatrix();
    //  OdomPQ.block<3, 1>(0, 3) = OdomP;
     
    // Eigen::Matrix4d  loam_RT2_lidar = Rtlb2la.inverse() *OdomPQ;

    // Eigen::Quaterniond  OdomQ_new(loam_RT2_lidar.block<3, 3>(0, 0) );
    // Eigen::Vector3d OdomP_new(loam_RT2_lidar.block<3, 1>(0, 3));


    // lidar_odomP  = OdomP;  //查看原始odom,发布出去
    // lidar_odomQ =  OdomQ;

     //WGPS_T_WVIO = lidar_T_W;

    //**********************************************************************
   

    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ_ENUBody;
    //globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
 
    Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP_ENUBody + WGPS_T_WVIO.block<3, 1>(0, 3);
    //Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    printf("new Global_odom: t: %f x: %f y: %f z:%f \n", t, globalP.x(), globalP.y(), globalP.z());
    std::cout<<"WGPS_T_WVIO"<<std::endl<<WGPS_T_WVIO<<std::endl;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";     //world
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);


    lidar_odomP  = globalP;  //查看原始odom---->转到 world下,发布出去
    lidar_odomQ =  globalQ;

    // lidar_odomP  = OdomP_ENUBody;  //查看原始odo,发布出去
    // lidar_odomQ =  OdomQ_ENUBody;

   globalPose_befOpt_Map[t] = globalPose;



    mPoseMap.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

/**
    just for debug
**/
void GlobalOptimization::get_lidar_odom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    // odomP = lastP;
    // odomQ = lastQ;
    odomP = lidar_odomP;
    odomQ = lidar_odomQ;
}




void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    if(simulation)
     {
        tmp[0] = tmp[0] + rng.gaussian ( w_sigma );
        tmp[1] = tmp[1] + rng.gaussian ( w_sigma );
        tmp[2] = tmp[2] + rng.gaussian ( w_sigma );
     }
    printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
	GPSPositionMap[t] = tmp;
    newGPS = true;

}
/**
**/
void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy_Lat,	double posAccuracy_Lon ,double altitude_Alt)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy_Lat ,posAccuracy_Lon , altitude_Alt};
    if(simulation)
     {
        tmp[0] = tmp[0] + rng.gaussian ( w_sigma );
        tmp[1] = tmp[1] + rng.gaussian ( w_sigma );
        tmp[2] = tmp[2] + rng.gaussian ( w_sigma );
     }
    printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
	GPSPositionMap[t] = tmp;
    newGPS = true;

}

/**

**/

void GlobalOptimization::input_Groundtruth(double t, double latitude, double longitude, double altitude)
{
    double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2]};
	Groundtruth_Map[t] = tmp;
}

void GlobalOptimization::inputIMU(double t ,Eigen::Quaterniond ImuQ)
{
	// double xyz[3];
	// GPS2XYZ(latitude, longitude, altitude, xyz);
	// vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    // printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
	// GPSPositionMap[t] = tmp;

    vector<double> Imu_map{ ImuQ.w(), ImuQ.x(), ImuQ.y(), ImuQ.z()};
    IMUOrientationMap[t] = Imu_map;
    newIMU = true;

}

void GlobalOptimization::optimize()
{

    while(true)
    {
        if(newGPS)
        {
            newGPS = false;
            printf("global optimization\n");
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            std::cout<<"localPoseMap.size()="<<localPoseMap.size()<<",globalPoseMap="<<globalPoseMap.size()<<std::endl;
            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS ,iterIMU;
            int i = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    //Eigen::Matrix4d iTj = lidar_T_W * wTi.inverse() * wTj;  //lidar_T_W 修改
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);

                    /*
                    double **para = new double *[4];
                    para[0] = q_array[i];
                    para[1] = t_array[i];
                    para[3] = q_array[i+1];
                    para[4] = t_array[i+1];

                    double *tmp_r = new double[6];
                    double **jaco = new double *[4];
                    jaco[0] = new double[6 * 4];
                    jaco[1] = new double[6 * 3];
                    jaco[2] = new double[6 * 4];
                    jaco[3] = new double[6 * 3];
                    vio_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[1]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[2]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[3]) << std::endl
                        << std::endl;
                    */

                }
                //gps factor
                double t = iterVIO->first;
                iterGPS = GPSPositionMap.find(t);
                if (iterGPS != GPSPositionMap.end())
                {
                    // ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                    //                                                    iterGPS->second[2], iterGPS->second[3]);

                    ceres::CostFunction* gps_function = TError2::Create(iterGPS->second[0], iterGPS->second[1],iterGPS->second[2], 
                                                                        iterGPS->second[3], iterGPS->second[4],iterGPS->second[5]);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(gps_function, loss_function, t_array[i]);

                    /*
                    double **para = new double *[1];
                    para[0] = t_array[i];

                    double *tmp_r = new double[3];
                    double **jaco = new double *[1];
                    jaco[0] = new double[3 * 3];
                    gps_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    */
                }


                // imu factor
                iterIMU = IMUOrientationMap.find(t);
                if(iterIMU != IMUOrientationMap.end())
                {
                    ceres::CostFunction* imu_function = Prior_RError::Create(iterIMU->second[0], iterIMU->second[1], 
                                                                       iterIMU->second[2], iterIMU->second[3],
                                                                       1);
                    problem.AddResidualBlock(imu_function, loss_function, q_array[i]);
 
                }
                

            }
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            // update global pose
            //mPoseMap.lock();
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
            	}
            }
            updateGlobalPath();
            updateGPSPath();                      //输出GPS轨迹
            update_globalPose_befOpt_MapPath();   //输出 gloab pose 未优化之前的轨迹
            update_localpose_MapPath();

            update_Groundtruth_MapPath();         //Groundtruth 参考值
            //printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
	return;
}



void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";   //world
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}


/**
    TODO: 更新GPS的轨迹
**/

void GlobalOptimization::updateGPSPath()
{
    gps_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = GPSPositionMap.begin(); iter != GPSPositionMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";   //world
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        
        pose_stamped.pose.orientation.w = 1;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;


        gps_path.header = pose_stamped.header;
        gps_path.poses.push_back(pose_stamped);
    }
}


/**
    TODO: 更新全局pose在未优化前的轨迹
**/

void GlobalOptimization::update_globalPose_befOpt_MapPath()
{
    global_befOpt_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPose_befOpt_Map.begin(); iter != globalPose_befOpt_Map.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";   //world
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        
        pose_stamped.pose.orientation.w = 1;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;


        global_befOpt_path.header = pose_stamped.header;
        global_befOpt_path.poses.push_back(pose_stamped);
    }
}

/**
    TODO:更新local odom的原始轨迹
**/

void GlobalOptimization::update_localpose_MapPath()
{
    local_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = localPoseMap.begin(); iter != localPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";   //world
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        
        pose_stamped.pose.orientation.w = 1;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;


        local_path.header = pose_stamped.header;
        local_path.poses.push_back(pose_stamped);
    }
}

void GlobalOptimization::update_Groundtruth_MapPath()
{
    Groundtruth_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = Groundtruth_Map.begin(); iter != Groundtruth_Map.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";   //world
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        
        pose_stamped.pose.orientation.w = 1;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;


        Groundtruth_path.header = pose_stamped.header;
        Groundtruth_path.poses.push_back(pose_stamped);
    }
}
/**
**/

void GlobalOptimization::set_sim_gps(const bool sim)
{
    simulation = sim;
}

void GlobalOptimization::set_sim_gpsnoise(const double noise)
{
    w_sigma = noise;
}
