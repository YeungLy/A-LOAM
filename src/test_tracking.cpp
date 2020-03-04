#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
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

#include "tracklet.h"
using std::atan2;
using std::cos;
using std::sin;

double timeObjectArray = 0;
double timeLaserOdometry= 0;


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



std::vector<DetectedBox> RosMsgToBoxes(jsk_recognition_msgs::BoundingBoxArrayConstPtr boxes_msg)
{

    std::vector<DetectedBox> boxes;
    for (size_t i = 0; i < boxes_msg->boxes.size(); ++i)
    {
        DetectedBox box;
        box.x = boxes_msg->boxes[i].pose.position.x;
        box.y = boxes_msg->boxes[i].pose.position.y;
        box.z = boxes_msg->boxes[i].pose.position.z;
        double qx = boxes_msg->boxes[i].pose.orientation.x;
        double qy = boxes_msg->boxes[i].pose.orientation.y;
        double qz = boxes_msg->boxes[i].pose.orientation.z;
        double qw = boxes_msg->boxes[i].pose.orientation.w;
        Eigen::Quaterniond q(qw, qx, qy, qz);
        //Z-Y-X 
        box.yaw = q.matrix().eulerAngles(2, 1, 0)(0);
        //ROS_INFO_STREAM("box msg pose orientation(x,y,z,w) : " << q.coeffs() << "\n yaw: " << box.yaw);
        box.l = boxes_msg->boxes[i].dimensions.x;
        box.w = boxes_msg->boxes[i].dimensions.y;
        box.h = boxes_msg->boxes[i].dimensions.z;

    }
    return boxes;
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

    nh.param<bool>("has_gt_objects", has_gt_objects);
    //string boxes_topic = "object_boxes";

    ros::Subscriber subObjectArray = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>("/object_boxes", 100, objectArrayHandler);

    //ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/odometry_gt", 100, laserOdometryHandler);
   
    
   
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        while (!objectArrayBuf.empty() && !laserOdometryBuf.empty())
        {
            mBuf.lock();
            while (!objectArrayBuf.empty() && objectArrayBuf.front()->header.stamp.toSec() < laserOdometryBuf.front()->header.stamp.toSec())
                objectArrayBuf.pop();
            if (objectArrayBuf.empty())
            {
                mBuf.unlock();
                break;
            }


            timeObjectArray = objectArrayBuf.front()->header.stamp.toSec();
            timeLaserOdometry = laserOdometryBuf.front()->header.stamp.toSec();
            if (timeObjectArray != timeLaserOdometry ) 
            {
                ROS_INFO("unsync message! time laserOdometry: %f, objectArray: %f", timeLaserOdometry, timeObjectArray);
                mBuf.unlock();
                break;
            }

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
            std::map<int, DetectedBox> objects = tracker.GetCurrentObjects();
            for (auto it = objects.begin(); it != objects.end(); ++it)
            {
                int id = it->first;
                DetectedBox box = it->second;
                Eigen::Vector3d t_lidar_obj(box.x, box.y, box.z);
                //w,x,y,z
                Eigen::AngleAxisd rz(box.yaw, Eigen::Vector3d::UnitZ());
                Eigen::AngleAxisd rx(0.0, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd ry(0.0, Eigen::Vector3d::UnitY());
                Eigen::Quaterniond q_lidar_obj = rz*ry*rx;
                Eigen::Quaterniond q_w_obj = q_w_lidar * q_lidar_obj;
                Eigen::Vector3d t_w_obj = q_w_lidar * t_lidar_obj + t_w_lidar;
                nav_msgs::Odometry objOdom;
                objOdom.header.frame_id = "/camera_init";
                objOdom.child_frame_id = "/object_odom";
                objOdom.header.stamp = ros::Time().fromSec(timeObjectArray);
                objOdom.pose.pose.orientation.x = q_w_obj.x();
                objOdom.pose.pose.orientation.y = q_w_obj.y();
                objOdom.pose.pose.orientation.z = q_w_obj.z();
                objOdom.pose.pose.orientation.w = q_w_obj.w();
                objOdom.pose.pose.position.x = t_w_obj.x();
                objOdom.pose.pose.position.y = t_w_obj.y();
                objOdom.pose.pose.position.z = t_w_obj.z();
                
                if (pubObjectsOdomDict.find(id) == pubObjectsOdomDict.end())
                {
                    ros::Publisher pubObjOdom = nh.advertise<nav_msgs::Odometry>("/obj_"+std::to_string(id)+"_odom", 3);
                    pubObjectsOdomDict[id] = pubObjOdom;
                }
                pubObjectsOdomDict[id].publish(objOdom);        
            }

        }
        rate.sleep();
    }
  
    return 0;
}
