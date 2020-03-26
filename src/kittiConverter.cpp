/*
 * Convert tracking result to kitti tracking label format so that we can use kitti evaluation tool 
 *
 * */

#include "kitti/utils.h"
#include "kitti/box.h"
#include "kitti/box_utils.h"
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <string>
#include <queue>
#include <sys/wait.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <mutex>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>


std::queue<jsk_recognition_msgs::BoundingBoxArrayConstPtr> tracksBuf;
std::mutex mBuf;

void tracksHandler(const jsk_recognition_msgs::BoundingBoxArrayConstPtr &tracks_msg)
{
    mBuf.lock();
    tracksBuf.push(tracks_msg);
    mBuf.unlock();
    
}

std::map<double, int> loadTimestampsToFrameIdxFile(std::string file)
{
    std::ifstream ifile(file, std::ifstream::in);
    std::map<double, int> time_to_frame;
    std::string line;
    int key_num = 0; 
    while (std::getline(ifile, line)) 
    {
        if (line.size() == 0)
            continue;
        std::stringstream ss(line);
        int id;
        double timestamp;
        ss >> id >> timestamp;
        time_to_frame[timestamp] = id;
        key_num++;
    }
    return time_to_frame; 
}
        


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_convert_result");
    ros::NodeHandle nh("~");
    std::string topic, calib_folder, label_path, timestamps_frameid_file;
    int num_frame = 5;
    nh.getParam("sub_topic", topic);
    nh.getParam("calib_folder", calib_folder);
    nh.getParam("label_path", label_path);
    nh.getParam("num_frame", num_frame);
    nh.getParam("timestamps_frameid_file", timestamps_frameid_file);
    std::cout << "param: " 
              << "topic: " << topic << ", calib_folder: " << calib_folder << ", label_path " << label_path << ", num_frame " << num_frame << std::endl; 

    std::map<double, int>time_to_frame = loadTimestampsToFrameIdxFile(timestamps_frameid_file);
    std::ofstream label_file(label_path, std::ofstream::out);

 
    std::string calib_path = calib_folder + "calib_velo_to_cam.txt";

    Eigen::Matrix<double, 4, 4> Tr_cam0_velo = loadCalibrationRigid(calib_path);
    Eigen::Matrix<double, 4, 4> R_rect_00 = loadCalibrationCamera("R_rect_00");
    Eigen::Matrix<double, 3, 4> P_rect_02 = loadCalibrationCamera("P_rect_02");
    Eigen::Matrix<double, 4, 4> T2 = Eigen::MatrixXd::Identity(4, 4);
    T2(0, 3) = P_rect_02(0, 3) / P_rect_02(0, 0);
    Eigen::Matrix<double, 4, 4> T_cam_velo = T2 * R_rect_00 * Tr_cam0_velo;
    //cam projection matrix
    Eigen::Matrix<double, 3, 3> K;
    K<<7.215377e+02, 0.000000e+00, 6.095593e+02, 
        0.000000e+00, 7.215377e+02, 1.728540e+02, 
        0.000000e+00, 0.000000e+00, 1.000000e+00;

    
    std::cout << "Output result from " << topic << " as kitti tracking label format at path: " << label_path << std::endl;

    ros::Subscriber subTracks = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>(topic, 100, tracksHandler);
    

    ros::Rate rate(100);
    
    //using timestamp to check frame..
    while (ros::ok())
    {
        ros::spinOnce();

        while (!tracksBuf.empty())
        {
            mBuf.lock();
            jsk_recognition_msgs::BoundingBoxArray tracks;
            tracks.header = tracksBuf.front()->header;
            tracks.boxes = tracksBuf.front()->boxes;
            tracksBuf.pop();
            mBuf.unlock();

            double timestamp = tracks.header.stamp.toSec();
            //clear all zero part at double.. double type using operator "==" may has some precision problem.
            std::stringstream ss;
            ss << timestamp;
            //std::cout << "timestamp str: " << ss.str() << std::endl;
            ss >> timestamp;
            //std::cout << "timestamp double: " << timestamp << std::endl;
            
            if (time_to_frame.find(timestamp) == time_to_frame.end())
            {
                //std::cout << "didnot found key!" << timestamp << std::endl;
                continue;
            }
            int frame_id = time_to_frame[timestamp];
            ROS_INFO_STREAM("[kittiConverter]processing boxes: " << tracks.boxes.size() << " to label file: " << label_path << ", at frame: " << frame_id ); 
            for (int i = 0; i < tracks.boxes.size(); ++i)
            {
                std::stringstream label;
                jsk_recognition_msgs::BoundingBox box = tracks.boxes[i];
                double qx = box.pose.orientation.x;
                double qy = box.pose.orientation.y;
                double qz = box.pose.orientation.z;
                double qw = box.pose.orientation.w;
                Eigen::Quaterniond q(qw, qx, qy, qz);
                double yaw = quaternion_to_euler(q)(2);
                kitti::Box3D box3d(box.pose.position.x, box.pose.position.y, box.pose.position.z, box.dimensions.x, box.dimensions.y, box.dimensions.z, yaw);
                //ROS_INFO_STREAM("box i: " << i << " from ros msg: " << box3d.getPrintString());
                //only keep in front of image plane
                Eigen::Matrix<double, 3, 8> corners3d_cam;
                bool visible_at_cam = transformVeloToCam3D(box3d, T_cam_velo, corners3d_cam);
                if (!visible_at_cam) 
                    continue;
                Box2D box2d = projectToImage(corners3d_cam, K);
                Eigen::Vector4d center3d_velo_homo(box3d.x, box3d.y, box3d.z, 1.0);
                Eigen::Vector4d center3d_cam_homo = T_cam_velo * center3d_velo_homo;
                assert(center3d_cam_homo(3) != 0);
                //std::cout << "center 3d cam homo: " << center3d_cam_homo << std::endl;
                double x_cam = center3d_cam_homo(0) / center3d_cam_homo(3);
                double y_cam = center3d_cam_homo(1) / center3d_cam_homo(3);
                double z_cam = center3d_cam_homo(2) / center3d_cam_homo(3);
                /*
                double x_cam = corners3d_cam.row(0).sum() / 8.0;
                double y_cam = corners3d_cam.block(1, 0, 1, 4).sum() / 4.0;
                double z_cam = corners3d_cam.row(2).sum() / 8.0;
                */
                double yaw_cam = box3d.yaw - M_PI / 2;
                int id = tracks.boxes[i].label;
                double score = tracks.boxes[i].value != 0 ? tracks.boxes[i].value : 0.9;
                //int id = 0;
                label << frame_id << " " << id
                      << " Car 0 0 0 " 
                      << box2d.xmin << " " << box2d.ymin << " " << box2d.xmax << " " << box2d.ymax << " "
                      << box3d.h << " " << box3d.w << " " << box3d.l << " "
                      << x_cam << " " << y_cam << " " << z_cam << " " << yaw_cam << " "
                      << score;
                //ROS_INFO_STREAM("[kittiConverter] frame: " << frame_id << ", row: " << i << ", label: " << label.str() );
                label_file << label.str() << "\n";
            }
        }
        rate.sleep();
    }

    /* write to file.*/
    label_file.close();
    return 0;

}
