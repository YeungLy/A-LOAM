// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <ceres/ceres.h>
#include <clocale>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <queue>
#include <mutex>
#include <thread>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "lidarFactor.hpp"
#include "ros/forwards.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"

using std::atan2;
using std::cos;
using std::sin;

bool systemInited = false;
double timeLaserCloud = 0;
double timeObjectArray = 0;
double timeLaserOdometry= 0;

int objNum = 0;

std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudBuf;
std::queue<nav_msgs::OdometryConstPtr> laserOdometryBuf;
std::queue<jsk_recognition_msgs::BoundingBoxArrayConstPtr> objectArrayBuf;

std::mutex mBuf;
jsk_recognition_msgs::BoundingBoxArrayPtr boxesLast(new jsk_recognition_msgs::BoundingBoxArray());
jsk_recognition_msgs::BoundingBoxArrayPtr boxesCurr(new jsk_recognition_msgs::BoundingBoxArray());
pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());

//lidar pose
Eigen::Quaterniond q_w_lidar(1, 0, 0, 0);
Eigen::Vector3d t_w_lidar(0, 0, 0);

std::map<uint32_t, ros::Publisher> pubObjectsOdomDict;
std::map<uint32_t, ros::Publisher> pubObjectsPathDict;
std::map<uint32_t, ros::Publisher> pubObjectsPathGTDict;
std::map<uint32_t, nav_msgs::Path > objects_path;
std::map<uint32_t, nav_msgs::Path > objects_path_gt;
bool has_gt_objects;
//std::map<int, nav_msgs::Odometry > objects_odom;

void matchBoxes()
{
    //box to trajectories
    //match last and curr by center points.
    
    //using motion model to estimate curr timestamp from last timestamp
    
    //N*M, or use Eigen::Matrix?
    bool * matched_last = new bool[boxesLast->boxes.size()];
    memset(matched_last, 0, sizeof(matched_last));
    int max_label = -1;
    ROS_INFO_STREAM("matching boxesCurr(size: " << boxesCurr->boxes.size() << " ) to boxesLast (size: " << boxesLast->boxes.size() << ")");
    for (size_t i_curr = 0; i_curr < boxesCurr->boxes.size(); ++i_curr)
    {
        //[TODO] maybe should use IoU !!
        double center_curr_x  = boxesCurr->boxes[i_curr].pose.position.x;
        double center_curr_y  = boxesCurr->boxes[i_curr].pose.position.y;
        double center_curr_z  = boxesCurr->boxes[i_curr].pose.position.z;
        double min_dist = 5;
        int matched_idx = -1;
        for (size_t i_last = 0; i_last < boxesLast->boxes.size(); ++i_last)
        {
            //should check whether this last box matched or not.
            if (matched_last[i_last])
                continue;

            double center_last_x  = boxesLast->boxes[i_last].pose.position.x;
            double center_last_y  = boxesLast->boxes[i_last].pose.position.y;
            double center_last_z  = boxesLast->boxes[i_last].pose.position.z;
            double dist = sqrt((center_curr_x - center_last_x) * (center_curr_x - center_last_x) +
                               (center_curr_y - center_last_y) * (center_curr_y - center_last_y) +
                               (center_curr_z - center_last_z) * (center_curr_z - center_last_z));

            //ROS_INFO_STREAM("i_curr: " << i_curr << ", position: (" << center_curr_x << ", " << center_curr_y << ", " << center_curr_z << ")");
            //ROS_INFO_STREAM("i_last: " << i_last << ", position: (" << center_last_x << ", " << center_last_y << ", " << center_last_z << ")");
            //ROS_INFO_STREAM("i_curr: " << i_curr << " , i_last: " << i_last << " , dist: " << dist);


            if (dist < min_dist)
            {
                min_dist = dist;
                matched_idx = i_last;
            }

        }

        if (matched_idx < 0)
        {//newcoming object. 
            boxesCurr->boxes[i_curr].label = objNum;
            ++objNum;
            //ROS_INFO_STREAM("new object!");
        }
        else
        {

            //ROS_INFO_STREAM("i_curr: " << i_curr << " , matched_idx: " << matched_idx << " , dist: " << min_dist);
            matched_last[matched_idx] = true;
            boxesCurr->boxes[i_curr].label = boxesLast->boxes[matched_idx].label;
        }

        //ROS_INFO_STREAM("i_curr: " << i_curr << " , label: " << boxesCurr->boxes[i_curr].label);
        //ROS_INFO_STREAM("i_curr: " << i_curr << " , gt-label: " << boxesCurr->boxes[i_curr].value);
    }
    
}
void trackObjects()
{

    //extract point cloud
    //for loop process each box.
    for (size_t i = 0; i < boxesCurr->boxes.size(); ++i)
    {
        jsk_recognition_msgs::BoundingBox box = boxesCurr->boxes[i];
        pcl::CropBox<PointType> boxFilter(true);
        boxFilter.setInputCloud(laserCloudIn);
        Eigen::Vector4f max_pt, min_pt;
        Eigen::Vector3f range{3.0, 3.0, 1.0};
        Eigen::Vector3f dimensions{box.dimensions.x, box.dimensions.y, box.dimensions.z};
        Eigen::Vector3f center{box.pose.position.x, box.pose.position.y, box.pose.position.z};
        max_pt << (dimensions+range)/2.0, 1.0;
        min_pt << -1*(dimensions+range)/2.0, 1.0;
        boxFilter.setMax(max_pt);
        boxFilter.setMin(min_pt);
        Eigen::Vector3d translation{box.pose.position.x, box.pose.position.y, box.pose.position.z};
        boxFilter.setTranslation(translation.cast<float>());
        Eigen::Quaterniond quaternion(box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w);
        Eigen::Vector3d euler_angles = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
        Eigen::Vector3d rotation{euler_angles(0), euler_angles(1), euler_angles(2)};
        boxFilter.setRotation(rotation.cast<float>());
        pcl::PointCloud<PointType>::Ptr objCloud(new pcl::PointCloud<PointType>());
        boxFilter.filter(*objCloud);
        ROS_INFO_STREAM("[trackObjects]i_curr: " << i 
                << ", box.label: " << box.label 
                <<",  segmented cloud points size: " << objCloud->points.size());

        double para_q[4] = {1, 0, 0, 0};
        double para_t[3] = {0, 0, 0};
        Eigen::Map<Eigen::Quaterniond> q_w_obj(para_q);
        Eigen::Map<Eigen::Vector3d> t_w_obj(para_t);
        ceres::LossFunction * loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization * q_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(para_q, 4, q_parameterization);
        problem.AddParameterBlock(para_t, 3);


        for (size_t p = 0; p < objCloud->points.size(); ++p)
        {
            Eigen::Vector3d p_lidar(objCloud->points[p].x, objCloud->points[p].y, objCloud->points[p].z);
            Eigen::Vector3d p_world, p_object;
            p_world = q_w_lidar * p_lidar + t_w_lidar;
            p_object = quaternion.inverse() * p_lidar - quaternion.inverse() * translation;
            ceres::CostFunction * cost_function = LidarDistanceFactor::Create(p_object, p_world);
            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        //options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        ROS_INFO_STREAM("[trackObjects]" << summary.BriefReport());
             //save msg

        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "/camera_init";
        odom_msg.child_frame_id = "/object_odom";
        odom_msg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odom_msg.pose.pose.orientation.x = q_w_obj.x();
        odom_msg.pose.pose.orientation.y = q_w_obj.y();
        odom_msg.pose.pose.orientation.z = q_w_obj.z();
        odom_msg.pose.pose.orientation.w = q_w_obj.w();
        odom_msg.pose.pose.position.x = t_w_obj.x();
        odom_msg.pose.pose.position.y = t_w_obj.y();
        odom_msg.pose.pose.position.z = t_w_obj.z();
        pubObjectsOdomDict[box.label].publish(odom_msg);

        if (objects_path.find(box.label) == objects_path.end())
        {
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "/camera_init";
            path_msg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            objects_path[box.label] = path_msg;
        }
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose.pose;
        pose_msg.header.stamp = odom_msg.header.stamp;
        objects_path[box.label].poses.push_back(pose_msg);
        pubObjectsPathDict[box.label].publish(objects_path[box.label]);

        if (has_gt_objects) 
        {
            uint32_t gt_label = box.value;
            if (objects_path_gt.find(gt_label) == objects_path_gt.end())
            {
                nav_msgs::Path path_gt_msg;
                path_gt_msg.header.frame_id = "/camera_init";
                path_gt_msg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                objects_path_gt[gt_label] = path_gt_msg;
            }
            Eigen::Quaterniond q_w_obj_gt = q_w_lidar * quaternion;
            Eigen::Vector3d t_w_obj_gt = q_w_lidar * translation + t_w_lidar;
            geometry_msgs::PoseStamped pose_gt_msg;
            pose_gt_msg.header = odom_msg.header;
            pose_gt_msg.pose.orientation.x = q_w_obj_gt.x();
            pose_gt_msg.pose.orientation.y = q_w_obj_gt.y();
            pose_gt_msg.pose.orientation.z = q_w_obj_gt.z();
            pose_gt_msg.pose.orientation.w = q_w_obj_gt.w();
            pose_gt_msg.pose.position.x = t_w_obj_gt.x();
            pose_gt_msg.pose.position.y = t_w_obj_gt.y();
            pose_gt_msg.pose.position.z = t_w_obj_gt.z();
            pose_gt_msg.header.stamp = odom_msg.header.stamp;
            objects_path_gt[gt_label].poses.push_back(pose_gt_msg);
            pubObjectsPathGTDict[gt_label].publish(objects_path_gt[box.label]);

            ROS_INFO_STREAM("[trackObjects]rotation angles: w_obj: " << q_w_obj.toRotationMatrix().eulerAngles(0, 1, 2) << ", rot angles: w_obj_gt: " << q_w_obj_gt.toRotationMatrix().eulerAngles(0, 1, 2) << ", at time: " << timeLaserOdometry);


        }

    }
    
    
    

}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mBuf.lock();
    laserCloudBuf.push(laserCloudMsg);
    mBuf.unlock();
}

void objectArrayHandler(const jsk_recognition_msgs::BoundingBoxArrayConstPtr &objectArrayMsg)
{
    mBuf.lock();
    objectArrayBuf.push(objectArrayMsg);
    mBuf.unlock();
}

void laserOdometryHandler(const nav_msgs::OdometryConstPtr &laserOdometryMsg)
{
    mBuf.lock();
    //save to buffer
    laserOdometryBuf.push(laserOdometryMsg);
    mBuf.unlock();
}

//match curr and last by boxes msg.
//assgin match result to label 
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracklets");
    ros::NodeHandle nh;
    int N_SCANS;
    nh.param<int>("scan_line", N_SCANS, 16);

    double MINIMUM_RANGE;
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    nh.param<bool>("has_gt_objects", has_gt_objects);
    //string boxes_topic = "object_boxes";

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_fg", 100, laserCloudHandler);
    ros::Subscriber subObjectArray = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>("/object_boxes", 100, objectArrayHandler);

    //ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/odometry_gt", 100, laserOdometryHandler);
   
    
   
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        while (!laserCloudBuf.empty() && !objectArrayBuf.empty() && !laserOdometryBuf.empty())
        {
            mBuf.lock();
            while (!laserCloudBuf.empty() && laserCloudBuf.front()->header.stamp.toSec() < laserOdometryBuf.front()->header.stamp.toSec())
                laserCloudBuf.pop();
            if (laserCloudBuf.empty())
            {
                mBuf.unlock();
                break;
            }
            while (!objectArrayBuf.empty() && objectArrayBuf.front()->header.stamp.toSec() < laserOdometryBuf.front()->header.stamp.toSec())
                objectArrayBuf.pop();
            if (objectArrayBuf.empty())
            {
                mBuf.unlock();
                break;
            }


            timeLaserCloud = laserCloudBuf.front()->header.stamp.toSec();
            timeObjectArray = objectArrayBuf.front()->header.stamp.toSec();
            timeLaserOdometry = laserOdometryBuf.front()->header.stamp.toSec();
            if (timeLaserCloud != timeObjectArray || timeLaserCloud != timeLaserOdometry ) 
            {
                ROS_INFO("unsync message! time laserCloud: %f, laserOdometry: %f, objectArray: %f", timeLaserCloud, timeLaserOdometry, timeObjectArray);
                mBuf.unlock();
                break;
            }

            laserCloudIn->clear();
            pcl::fromROSMsg(*laserCloudBuf.front(), *laserCloudIn);
            laserCloudBuf.pop();

            jsk_recognition_msgs::BoundingBoxArrayPtr objectArrayIn(new jsk_recognition_msgs::BoundingBoxArray());
            boxesCurr->header = objectArrayBuf.front()->header;
            boxesCurr->boxes = objectArrayBuf.front()->boxes;
            objectArrayBuf.pop();

            q_w_lidar.x() = laserOdometryBuf.front()->pose.pose.orientation.x;
            q_w_lidar.y() = laserOdometryBuf.front()->pose.pose.orientation.y;
            q_w_lidar.z() = laserOdometryBuf.front()->pose.pose.orientation.z;
            q_w_lidar.w() = laserOdometryBuf.front()->pose.pose.orientation.w;
            t_w_lidar.x() = laserOdometryBuf.front()->pose.pose.position.x;
            t_w_lidar.y() = laserOdometryBuf.front()->pose.pose.position.y;
            t_w_lidar.z() = laserOdometryBuf.front()->pose.pose.position.z;
            laserOdometryBuf.pop();
            

            mBuf.unlock();

            TicToc t_whole;
            //Filter pointcloud using crop box. dimension limit by diagonal line of box.
            if (!systemInited)
            {
                systemInited = true;
                if (objNum == 0) 
                {
                        //add object label for first frame.
                        
                    for (size_t i = 0; i < boxesCurr->boxes.size(); ++i)
                    {
                        boxesCurr->boxes[i].label = i;

                        ++objNum;
                        ros::Publisher pubObjectOdometry = nh.advertise<nav_msgs::Odometry>("/obj_"+std::to_string(i)+"_odom", 100);
                        pubObjectsOdomDict[i] = pubObjectOdometry;
                        ros::Publisher pubObjectPath = nh.advertise<nav_msgs::Path>("/obj_"+std::to_string(i)+"_path", 100);
                        pubObjectsPathDict[i] = pubObjectPath;
                        if (has_gt_objects)
                        {
                            ros::Publisher pubObjectPathGT = nh.advertise<nav_msgs::Path>("/obj_"+std::to_string(i)+"_path_gt", 100);
                            pubObjectsPathGTDict[i] = pubObjectPathGT;
                        }
                    }
                    ROS_INFO_STREAM("Initializate tracklets number " << pubObjectsOdomDict.size());
                } 
            }
            else 
            {
                matchBoxes();
                for (size_t i = 0; i < boxesCurr->boxes.size(); ++i)
                {
                    uint32_t label = boxesCurr->boxes[i].label;
                    //if (label == pubObjectsOdomDict.size())
                    if (pubObjectsOdomDict.find(label) == pubObjectsOdomDict.end())
                    {
                        ros::Publisher pubObjectOdometry = nh.advertise<nav_msgs::Odometry>("/obj_"+std::to_string(label)+"_odom", 100);
                        pubObjectsOdomDict[label] = pubObjectOdometry;
                        ros::Publisher pubObjectPath = nh.advertise<nav_msgs::Path>("/obj_"+std::to_string(label)+"_path", 100);
                        pubObjectsPathDict[label] = pubObjectPath;
                    }               

                    if (has_gt_objects)
                    {
                        uint32_t gt_label = boxesCurr->boxes[i].value;
                        if (pubObjectsPathGTDict.find(gt_label) == pubObjectsPathGTDict.end())
                        {
                            ros::Publisher pubObjectPathGT = nh.advertise<nav_msgs::Path>("/obj_"+std::to_string(i)+"_path_gt", 100);
                            pubObjectsPathGTDict[gt_label] = pubObjectPathGT;
                        }
                                       
                    }

                }
                trackObjects();

            
            }
            //update last
            boxesLast->header = boxesCurr->header;
            boxesLast->boxes = boxesCurr->boxes;
            
            //publish

        }
        rate.sleep();
    }
  
    return 0;
}
