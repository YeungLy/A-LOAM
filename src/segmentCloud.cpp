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


#include <cmath>
#include <cwchar>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
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

using std::atan2;
using std::cos;
using std::sin;

bool systemInited = false;
double timeLaserCloud = 0;
double timeObjectArray = 0;


std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudBuf;
std::queue<jsk_recognition_msgs::BoundingBoxArrayConstPtr> objectArrayBuf;
std::mutex mBuf;



void segmentOutBoxes(pcl::PointCloud<PointType>::ConstPtr laserCloudIn, jsk_recognition_msgs::BoundingBoxArrayConstPtr boxesMsg, pcl::PointCloud<PointType>::Ptr bgCloudOut, pcl::PointCloud<PointType>::Ptr fgCloudOut)
{

    //pcl::copyPointCloud(*laserCloudIn, *bgCloudOut);
    ROS_INFO_STREAM("[segmentOutBoxes]boxes size: " << boxesMsg->boxes.size());
    std::vector<pcl::PointCloud<PointType>::ConstPtr > objectsCloud;

    uint32_t fgPointNum = 0;
    for (size_t obj_idx = 0; obj_idx < boxesMsg->boxes.size(); obj_idx++) {
        jsk_recognition_msgs::BoundingBox box = boxesMsg->boxes[obj_idx];
       
        //if (box.label != test_obj_id)
        //    continue;
        pcl::CropBox<PointType> boxFilter(true);
        boxFilter.setInputCloud(bgCloudOut);
        Eigen::Vector4f max_pt, min_pt;
        Eigen::Vector3f range{3.0, 3.0, 1.0};
        Eigen::Vector3f dimensions{box.dimensions.x, box.dimensions.y, box.dimensions.z};
        Eigen::Vector3f center{box.pose.position.x, box.pose.position.y, box.pose.position.z};
        max_pt << (dimensions+range)/2.0, 1.0;
        min_pt << -1*(dimensions+range)/2.0, 1.0;
        /*
        float radius_xy = sqrt(box.dimensions.x * box.dimensions.x + box.dimensions.y * box.dimensions.y);
        xmax= box.pose.position.x + radius_xy / 2.0;
        ymax = box.pose.position.y + radius_xy / 2.0;
        zmax = box.pose.position.z;
        xmin = box.pose.position.x - radius_xy / 2.0;
        ymin = box.pose.position.y - radius_xy / 2.0;
        zmin = box.pose.position.z - box.dimensions.z;
        */
        //std::cout << "max pt: " << max_pt << "\nmin_pt: " << min_pt << std::endl;
        boxFilter.setMax(max_pt);
        boxFilter.setMin(min_pt);
        Eigen::Vector3f translation{box.pose.position.x, box.pose.position.y, box.pose.position.z};
        boxFilter.setTranslation(translation);
        Eigen::Quaterniond quaternion(box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w);
        Eigen::Vector3d euler_angles = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
        Eigen::Vector3f rotation{euler_angles(0), euler_angles(1), euler_angles(2)};
        //Eigen::Vector3f rotation{box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z};
        boxFilter.setRotation(rotation);
        pcl::PointCloud<PointType>::Ptr objCloud(new pcl::PointCloud<PointType>());
        boxFilter.filter(*objCloud);
        objectsCloud.push_back(objCloud);
        ROS_INFO_STREAM("obj cloud " << obj_idx <<" indices size: " << objCloud->size());
        fgPointNum += objCloud->size();
        pcl::IndicesConstPtr bg_point_indices =  boxFilter.getRemovedIndices();
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(bgCloudOut);
        extract.setIndices(bg_point_indices);
        extract.filter(*bgCloudOut);
        ROS_INFO_STREAM("removed bg point size: " << bg_point_indices->size());
    }

    fgCloudOut->height = 1;
    fgCloudOut->width = fgPointNum;
   
    fgCloudOut->points.resize(fgPointNum);
    
    size_t fg_idx = 0;
    for (size_t obj_idx = 0; obj_idx < objectsCloud.size(); ++obj_idx)
    {
        for (size_t p = 0; p < objectsCloud[obj_idx]->size(); ++p)
        {
            fgCloudOut->points[fg_idx] = objectsCloud[obj_idx]->points[p]; 
            ++fg_idx;
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "segmentCloud");
    ros::NodeHandle nh;

    /*
    int N_SCANS;
    nh.param<int>("scan_line", N_SCANS, 16);

    double MINIMUM_RANGE;
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);
    */

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);
    ros::Subscriber subObjectArray = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>("/object_boxes", 100, objectArrayHandler);
   

    ros::Publisher pubLaserCloud;
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_bg", 100);
    ros::Publisher pubObjectsCloud;
    pubObjectsCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_fg", 100);

    pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr bgCloudOut(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr fgCloudOut(new pcl::PointCloud<PointType>());
   
    //publish bg velodyne points and obj velodyne points
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!laserCloudBuf.empty() && !objectArrayBuf.empty()) {
            timeLaserCloud = laserCloudBuf.front()->header.stamp.toSec();
            timeObjectArray = objectArrayBuf.front()->header.stamp.toSec();
            if (timeLaserCloud != timeObjectArray) {
                ROS_INFO("unsync message! time laserCloud: %f, time objectArray: %f", timeLaserCloud, timeObjectArray);
                ROS_BREAK();
                
            }

            //segment point cloud
            mBuf.lock();
            laserCloudIn->clear();
            pcl::fromROSMsg(*laserCloudBuf.front(), *laserCloudIn);
            laserCloudBuf.pop();
            jsk_recognition_msgs::BoundingBoxArrayPtr objectArrayIn(new jsk_recognition_msgs::BoundingBoxArray());
            objectArrayIn->header = objectArrayBuf.front()->header;
            objectArrayIn->boxes = objectArrayBuf.front()->boxes;
            objectArrayBuf.pop();

            bgCloudOut->clear();
            fgCloudOut->clear();

            mBuf.unlock();

            pcl::copyPointCloud(*laserCloudIn, *bgCloudOut);
            TicToc t_whole;
            //Filter pointcloud using crop box. dimension limit by diagonal line of box.
            segmentOutBoxes(laserCloudIn, objectArrayIn, bgCloudOut, fgCloudOut);
            //publish
            ROS_INFO_STREAM("publishing velodyne_points_bg msg at segmentCloud, cloud size: " << bgCloudOut->size());
            sensor_msgs::PointCloud2 bgCloudOutMsg;
            pcl::toROSMsg(*bgCloudOut, bgCloudOutMsg);
            bgCloudOutMsg.header.stamp = ros::Time().fromSec(timeLaserCloud);
            bgCloudOutMsg.header.frame_id = "/camera_init";
            pubLaserCloud.publish(bgCloudOutMsg);
            
            sensor_msgs::PointCloud2 fgCloudOutMsg;
            pcl::toROSMsg(*fgCloudOut, fgCloudOutMsg);
            fgCloudOutMsg.header.stamp = ros::Time().fromSec(timeLaserCloud);
            fgCloudOutMsg.header.frame_id = "/camera_init";
            pubObjectsCloud.publish(fgCloudOutMsg);

            
        }
        rate.sleep();
        
    }
    return 0;
}
