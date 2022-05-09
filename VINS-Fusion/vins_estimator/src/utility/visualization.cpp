/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"
#include "gnss_tools.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include<tf/transform_datatypes.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;
ros::Publisher pub_odometrySpan;
ros::Subscriber span_BP_sub;// subscribe the topic of Span
ros::Subscriber VINS_GNSS_FG_sub;

ros::Publisher pub_image_track;


nav_msgs::Odometry odometry_G;
nav_msgs::Odometry spanOdom;
GNSS_Tools gnss_tools_1; 
bool flag=0;

Eigen::MatrixXd original1;
double original_heading =0;
double span_heading =0;
double vins_heading =0;

CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;

std_msgs::Header g_odometry_header;
int time_now =0;
nav_msgs::Odometry odometry_0;

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);
void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);

std::string groundTruthFolder = "../catkin/src/result/groundTruth.csv";

void span_bp_callback(const novatel_msgs::INSPVAXConstPtr& fix_msg)
{
    std::cout << std::setprecision(12);

    Eigen::MatrixXd laloal;
    laloal.resize(3,1);
    laloal(0) = fix_msg->longitude;
    laloal(1) = fix_msg->latitude;
    laloal(2) = fix_msg->altitude;
    
    Eigen::MatrixXd ecef;
    ecef.resize(3,1);
    ecef = gnss_tools_1.llh2ecef(laloal);

    original1.resize(3,1);

    if(flag==0) // initial value
    {
      flag = 1;
      original1(0) = laloal(0);
      original1(1) = laloal(1);
      original1(2) = laloal(2);
      original_heading = fix_msg->azimuth;
    }
    span_heading = fix_msg->azimuth;

    Eigen::MatrixXd enu1;
    enu1.resize(3,1);
    enu1 = gnss_tools_1.ecef2enu(original1,ecef);

    /*aligning the gps heading and the vio heading in the same coordinate*/
    double prex_ = enu1(0);
    double prey_ = enu1(1);
    double theta = (original_heading)*(3.141592/180.0);
    enu1(0) = prex_*cos(theta) - prey_*sin(theta);
    enu1(1) = prex_*sin(theta) + prey_*cos(theta);
    std::cout<<"enu1 ------->: " << enu1 << "\n";
    

    spanOdom.header.frame_id = "world"; // pose
    spanOdom.child_frame_id = "world"; // twist
    spanOdom.pose.pose.position.x = enu1(0);
    spanOdom.pose.pose.position.y = enu1(1);
    spanOdom.pose.pose.position.z = 0;
    pub_odometrySpan.publish(spanOdom);

    spanOdom.twist.twist.linear.x = fix_msg->east_velocity;
    spanOdom.twist.twist.linear.y = fix_msg->north_velocity;

    double pre_x_ = spanOdom.twist.twist.linear.x;
    double pre_y_ =spanOdom.twist.twist.linear.y;
    double theta_ = (original_heading)*(3.141592 / 180.0);

    spanOdom.twist.twist.linear.x = pre_x_*cos(theta_) - pre_y_*sin(theta_);
    spanOdom.twist.twist.linear.y = pre_x_*sin(theta_) + pre_y_*cos(theta_);

    double roll = fix_msg->roll;
    double pitch = fix_msg->pitch;
    /*heading changes of Span*/
    double yaw = -(fix_msg->azimuth-original_heading);

    //transform the euclidean to quanternion
    tf2::Quaternion gt_q;
    gt_q.setRPY(roll * 3.1415926/180, pitch * 3.1415926/180, yaw * 3.1415926/180);
    gt_q.normalize();

    time_now = time_now+1;
    
    //write to groundTruth
    ofstream foutG(groundTruthFolder, ios::app);
    foutG.setf(ios::fixed, ios::floatfield);
    foutG.precision(10);
    foutG << time_now << " "; // check again
    foutG.precision(5);
    foutG << enu1(0) << " "
          << enu1(1) << " "
          << enu1(2) << " "
          << gt_q[0] << " "
          << gt_q[1] << " "
          << gt_q[2] << " "
          << gt_q[3] << endl;
    foutG.close();

    //write to vio
    ofstream foutC(VINS_RESULT_PATH, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(10);
    // foutC << header.stamp.toSec() * 1e9 << ",";
    // g_odometry_header = header;
    //  foutC << header.stamp.toSec()<< " "; 
    foutC << time_now<< " "; 
    foutC.precision(5);
    foutC << odometry_0.pose.pose.position.x << " " // using space to separate the value, original: ","
            << odometry_0.pose.pose.position.y << " "
            << odometry_0.pose.pose.position.z << " "
            << odometry_0.pose.pose.orientation.x << " "
            << odometry_0.pose.pose.orientation.y << " "
            << odometry_0.pose.pose.orientation.z << " "
            << odometry_0.pose.pose.orientation.w << endl; 
          //  << g_pdop << endl;  
        //   << estimator.Vs[WINDOW_SIZE].x() << ","
        //   << estimator.Vs[WINDOW_SIZE].y() << ","
        //   << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
    foutC.close();
    printf("time: %f, t: %f %f %f q: %f %f %f %f \n",time_now, odometry_0.pose.pose.position.x, odometry_0.pose.pose.position.y, odometry_0.pose.pose.position.z,
                                                    odometry_0.pose.pose.orientation.x, odometry_0.pose.pose.orientation.y, odometry_0.pose.pose.orientation.z, odometry_0.pose.pose.orientation.w);

}

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_odometrySpan = n.advertise<nav_msgs::Odometry>("odometryenu", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);

    span_BP_sub =n.subscribe("/novatel_data/inspvax", 500, span_bp_callback);
    VINS_GNSS_FG_sub = n.subscribe("/globalEstimator/global_odometry",500,VINS_GNSS_FG);

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);
}

void VINS_GNSS_FG(const nav_msgs::OdometryConstPtr& odom_msg)
{
   const ros::Time& stamp = odom_msg->header.stamp; // ros time 
   odometry_G = *odom_msg; 
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}

void pubTrackImage(const cv::Mat &imgTrack, const double t)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track.publish(imgTrackMsg);
}


void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    if (ESTIMATE_EXTRINSIC)
    {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            //ROS_DEBUG("calibration result for camera %d", i);
            ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
            ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
            
            cout << "extrinsic tic"  << estimator.tic[i].transpose()<<endl;
            cout <<  "extrinsic ric"   << Utility::R2ypr(estimator.ric[i]).transpose()<<endl;

            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.ric[i];
            eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            if(i == 0)
                fs << "body_T_cam0" << cv_T ;
            else
                fs << "body_T_cam1" << cv_T ;
        }
        fs.release();
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();

        odometry_0 = odometry;

        double quatx =odometry.pose.pose.orientation.x;
        double quaty =odometry.pose.pose.orientation.y;
        double quatz =odometry.pose.pose.orientation.z;
        double quatw =odometry.pose.pose.orientation.w;

        tf::Quaternion q(quatx,quaty,quatz,quatw);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        vins_heading = yaw*(-1)*(180/3.14);
        if(vins_heading<0)
        {
          vins_heading = vins_heading+360;          

        }

        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        // write result to file
        // ofstream foutC(VINS_RESULT_PATH, ios::app);
        // foutC.setf(ios::fixed, ios::floatfield);
        // foutC.precision(0);
        // foutC << header.stamp.toSec() * 1e9 << ",";
        // foutC.precision(5);
        // foutC << estimator.Ps[WINDOW_SIZE].x() << ","
        //       << estimator.Ps[WINDOW_SIZE].y() << ","
        //       << estimator.Ps[WINDOW_SIZE].z() << ","
        //       << tmp_Q.w() << ","
        //       << tmp_Q.x() << ","
        //       << tmp_Q.y() << ","
        //       << tmp_Q.z() << ","
        //       << estimator.Vs[WINDOW_SIZE].x() << ","
        //       << estimator.Vs[WINDOW_SIZE].y() << ","
        //       << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        // foutC.close();
        // Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
        // printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x(), tmp_T.y(), tmp_T.z(),
        //                                                   tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        if(STEREO)
        {
            Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
            Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
            cameraposevisual.add_pose(P, R);
        }
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}


void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;


    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}


void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);

}

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);


        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        point_cloud.header.frame_id = "world";
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point.publish(point_cloud);
    }
}