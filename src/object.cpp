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
#include "tracking/tracklet.h"
#include "tracking/utils.h"
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
jsk_recognition_msgs::BoundingBoxArrayPtr boxesCurr(new jsk_recognition_msgs::BoundingBoxArray());
pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());

TrackletManager tracker;


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

            tracker.Update(RosMsgToBoxes(boxesCurr));

            
            //publish
            std::map<uint32_t, DetectedBox> objects = tracker.GetCurrentObjects();
            for (auto it = objects.begin(); it != objects.end(); ++it)
            {
                uint32_t id = it->first();
                DetectedBox box = it->second();
                Eigen::Vector3d t_lidar_obj(box.x, box.y, box.z);
                //w,x,y,z
                Eigen::Quaterniond q_lidar_obj(cos(box.yaw/2), 0.0, sin(box.yaw/2), 0.0);
                Eigen::Quaterniond q_w_obj = q_w_lidar * q_lidar_obj;
                Eigen::Vector3d t_w_obj = q_w_lidar * t_lidar_obj + t_w_lidar;
                nav_msgs::Odometry objOdom;
                objOdom.header.frame_id = "/camera_init";
                objOdom.header.stamp = ros::Time().fromeSec(timeObjectArray);
                objOdom.pose.pose.orientation.x = q_w_obj.x();
                objOdom.pose.pose.orientation.y = q_w_obj.y();
                objOdom.pose.pose.orientation.z = q_w_obj.z();
                objOdom.pose.pose.orientation.w = q_w_obj.w();
                objOdom.pose.pose.position.x = t_w_obj.x();
                objOdom.pose.pose.position.y = t_w_obj.y();
                objOdom.pose.pose.position.z = t_w_obj.z();
                
                if (pubObjectsOdomDict.find(id) == pubObjectsOdomDict.end())
                {
                    ros::Publisher pubObjOdom = nh.advertise<nav_msgs::Odometry>("/obj_"+id+"_odom", 3);
                    pubObjectsOdomDict[id] = pubObjOdom;
                }
                pubObjectsOdomDict[id].publish(objOdom);        
            }

        }
        rate.sleep();
    }
  
    return 0;
}
