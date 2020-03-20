#include <vector>
#include <string>
#include <Eigen/Dense>
#include "Eigen/src/Geometry/AngleAxis.h"
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

#include "tracker.h"
#include "kitti/utils.h"
#include "kitti/box_utils.h"

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


//lidar pose
Eigen::Quaterniond q_w_lidar(1, 0, 0, 0);
Eigen::Vector3d t_w_lidar(0, 0, 0);

std::map<int, ros::Publisher> pubObjectsOdomDict;
std::map<int, ros::Publisher> pubObjectsPathDict;
std::map<int, ros::Publisher> pubObjectsPathGTDict;
std::map<int, nav_msgs::Path > objects_path_msg;
std::map<int, nav_msgs::Path > objects_path_gt_msg;
bool use_gt_objects;
//std::map<int, nav_msgs::Odometry > objects_odom;




void BBoxArrayMsgToBoxes(jsk_recognition_msgs::BoundingBoxArrayConstPtr boxes_msg, bool has_label, std::vector<DetectedBox> & boxes)
{

    Eigen::Vector3d euler;
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
        euler = quaternion_to_euler(q);
        box.yaw = euler(2);
        //ROS_INFO_STREAM("box msg pose orientation(x,y,z,w) : " << q.coeffs() << "\n yaw: " << box.yaw);
        //
        box.l = boxes_msg->boxes[i].dimensions.x;
        box.w = boxes_msg->boxes[i].dimensions.y;
        box.h = boxes_msg->boxes[i].dimensions.z;
        if (has_label)
            box.id = boxes_msg->boxes[i].label;
        else
            box.id = -1;
        boxes.push_back(box);

    }
}

void BoxToBBoxMsg(const DetectedBox & box, jsk_recognition_msgs::BoundingBox & bbox_msg)
{
    Eigen::Vector3d euler(0.0, 0.0, box.yaw);
    Eigen::Quaterniond q = euler_to_quaternion(euler);
    bbox_msg.pose.orientation.x = q.x();
    bbox_msg.pose.orientation.y = q.y();
    bbox_msg.pose.orientation.z = q.z();
    bbox_msg.pose.orientation.w = q.w();
    bbox_msg.pose.position.x = box.x;
    bbox_msg.pose.position.y = box.y;
    bbox_msg.pose.position.z = box.z;
    bbox_msg.dimensions.x = box.l;
    bbox_msg.dimensions.y = box.w;
    bbox_msg.dimensions.z = box.h;
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

    bool verbose_tracker;
    nh.getParam("use_gt_objects", use_gt_objects);
    nh.getParam("verbose_tracker", verbose_tracker);
    bool from_param;
    nh.param<bool>("use_gt_objects", from_param, false);
    string boxes_topic = "object_boxes";
    if (use_gt_objects)
        boxes_topic = "tracks_gt";
    if (from_param)
        ROS_INFO_STREAM("from_param is true");
    else 
        ROS_INFO_STREAM("from_param is false");


    int loglevel = 0;
    if (!verbose_tracker)
        loglevel = 1;

    Tracker tracker(loglevel);
       
    ROS_INFO_STREAM("using boxes_topic: " << boxes_topic);
    //ros::Subscriber subObjectArray = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>("/object_boxes", 100, objectArrayHandler);
    ros::Subscriber subObjectArray = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>(boxes_topic, 100, objectArrayHandler);

    //ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/odometry_gt", 100, laserOdometryHandler);


    ros::Publisher pubTracks = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/tracks", 100);
   
    
   
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

            std::vector<DetectedBox> detections;
            BBoxArrayMsgToBoxes(boxesCurr, use_gt_objects, detections);
            if (use_gt_objects)
                tracker.UpdateGT(detections);
            else
                tracker.Update(detections);

            
            jsk_recognition_msgs::BoundingBoxArray tracks_msg;
            tracks_msg.header.frame_id = "/camera_init";
            tracks_msg.header.stamp = ros::Time().fromSec(timeObjectArray);
                
            //publish
            std::map<int, DetectedBox> objects = tracker.GetCurrentObjects();
            for (auto it = objects.begin(); it != objects.end(); ++it)
            {
                int id = it->first;
                DetectedBox box = it->second;
                if (id < 0 || box.isValid() == false)
                {
                    ROS_ERROR_STREAM("[publish] There are invalid id or box at current tracker!");
                    continue;
                }
                //publish current tracks(current tracked boxes), relative to current sensor frame.
                jsk_recognition_msgs::BoundingBox bbox_msg;
                BoxToBBoxMsg(box, bbox_msg);
                bbox_msg.label = id;
                bbox_msg.header.stamp = tracks_msg.header.stamp;
                bbox_msg.header.frame_id = tracks_msg.header.frame_id;
                tracks_msg.boxes.push_back(bbox_msg);
                ROS_INFO_STREAM("[test_tracking]publish tracks_msg, id: " << id << ", at frame: " << tracker.frame_idx_);

                //publish each object' odom and path, relative to global frame..

                Eigen::Vector3d t_lidar_obj(box.x, box.y, box.z);
                //w,x,y,z
                Eigen::Vector3d euler(0.0, 0.0, box.yaw);
                Eigen::Quaterniond q_lidar_obj = euler_to_quaternion(euler);
                Eigen::Quaterniond q_w_obj = q_w_lidar * q_lidar_obj;
                Eigen::Vector3d t_w_obj = q_w_lidar * t_lidar_obj + t_w_lidar;

                //publish odometry
                if (pubObjectsOdomDict.find(id) == pubObjectsOdomDict.end())
                {
                    ros::Publisher pubObjOdom = nh.advertise<nav_msgs::Odometry>("/obj_"+std::to_string(id)+"_odom", 3);
                    pubObjectsOdomDict[id] = pubObjOdom;
                }

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
                pubObjectsOdomDict[id].publish(objOdom);        

                //publish path
                if (pubObjectsPathDict.find(id) == pubObjectsPathDict.end())
                {
                    ros::Publisher pubObjPath = nh.advertise<nav_msgs::Path>("/obj_"+std::to_string(id)+"_path", 3);
                    pubObjectsPathDict[id] = pubObjPath;
                    nav_msgs::Path path_msg;
                    path_msg.header.frame_id = "/camera_init";
                    path_msg.header.stamp = ros::Time().fromSec(timeObjectArray);
                    objects_path_msg[id] = path_msg;
                }

                geometry_msgs::PoseStamped objPose;
                objPose.header = objOdom.header;
                objPose.pose = objOdom.pose.pose;
                objPose.header.stamp = objOdom.header.stamp;
                //objects_path_msg[id].header.stamp = objOdom.header.stamp;
                objects_path_msg[id].poses.push_back(objPose);

                pubObjectsPathDict[id].publish(objects_path_msg[id]);        
                
            }
            pubTracks.publish(tracks_msg);

        
        }
        //ROS_INFO_STREAM("Wating for next frame..");
        rate.sleep();
    }
  
    return 0;
}
