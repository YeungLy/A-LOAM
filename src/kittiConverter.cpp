/*
 * Convert tracking result to kitti tracking label format so that we can use kitti evaluation tool 
 *
 * */

#include "kitti/utils.h"
#include "kitti/box_utils.h"
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <string>
#include <queue>
#include <vector>
#include <iostream>
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
int frame_idx;

void tracksHandler(const jsk_recognition_msgs::BoundingBoxArrayConstPtr &tracks_msg)
{
    mBuf.lock();
    tracksBuf.push(tracks_msg);
    mBuf.unlock();
    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_convert_result");
    ros::NodeHandle nh("~");
    std::string topic, calib_folder, output_folder;
    int num_frame = 5;
    bool output_timestamps;
    nh.getParam("sub_topic", topic);
    nh.getParam("calib_folder", calib_folder);
    nh.getParam("output_folder", output_folder);
    nh.getParam("num_frame", num_frame);
    nh.getParam("output_timestamps", output_timestamps);
    std::cout << "param: " 
              << "topic: " << topic << ", calib_folder: " << calib_folder << ", output_folder " << output_folder << ", num_frame " << num_frame << ", output_timestamps " << output_timestamps << std::endl;

    std::stringstream label_path;
    label_path << output_folder << "0001.txt"; 
    std::ofstream label_file(label_path.str(), std::ofstream::out);
 
    std::string calib_path = calib_folder + "calib_velo_to_cam.txt";

    Eigen::Matrix<double, 4, 4> Tr_cam0_velo = loadCalibrationRigid(calib_path);

    Eigen::Matrix<double, 4, 4> R_rect_00 = loadCalibrationCamera("R_rect_00");
    Eigen::Matrix<double, 4, 4> T_cam0_velo = R_rect_00 * Tr_cam0_velo;

    Eigen::Matrix<double, 3, 4> P_rect_02 = loadCalibrationCamera("P_rect_02");
    
    frame_idx = 0;

    std::cout << "Output result from " << topic << " as kitti tracking label format at path: " << label_path.str() << std::endl;

    ros::Subscriber subTracks = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>(topic, 100, tracksHandler);
    
    std::map<int, std::vector<std::string> > all_tracks_labels;

    ros::Rate rate(100);
    
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

            if (frame_idx == num_frame)
                break;

            Eigen::Matrix<double, 7, 1> box3d;
            Eigen::Matrix<double, 4, 1> box2d;
            ROS_INFO_STREAM("[kittiConverter]processing boxes: " << tracks.boxes.size() << " to label file: " << label_path.str() ); 
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
                
                box3d << box.pose.position.x, box.pose.position.y, box.pose.position.z,
                         box.dimensions.x, box.dimensions.y, box.dimensions.z,
                         yaw;
                //center at velo
                Eigen::Vector4d box_center_homo(box3d(0), box3d(1), box3d(2), 1.0);
                //velo to cam0, update (x,y,z) at camera frame at box3d 
                box_center_homo = T_cam0_velo * box_center_homo;
                box_center_homo /= box_center_homo(3);
                box3d.topRows(3) = box_center_homo.topRows(3);
                // convert rz at velo to ry at cam
                box3d(6) -= M_PI/2;
                //box3d to 8 corners 
                Eigen::Matrix<double, 3, 8> box3d_corners = convertBox3Dto8corners(box3d,"camera");
                Eigen::Matrix<double, 2, 8> projected_box3d;
                //project from cam0 to cam2, left color image
                projectBoxtoImage(box3d_corners, P_rect_02, projected_box3d);
                //box2d, left, top, right, bottom
                double left = projected_box3d.row(0).minCoeff();
                double right = projected_box3d.row(0).maxCoeff();
                double top = projected_box3d.row(1).minCoeff();
                double bottom = projected_box3d.row(1).maxCoeff();
                int id = tracks.boxes[i].label;
                //int id = 0;
                label << frame_idx << " " << id
                      << " Car 0 0 0 " 
                      << left << " " << top << " " << right << " " << bottom << " "
                      << box3d(5) << " " << box3d(4) << " " << box3d(3) << " "
                      << box3d(0) << " " << box3d(1) << " " << box3d(2) << " "
                      << box3d(6) <<" 0.9";
                ROS_INFO_STREAM("[kittiConverter] frame: " << frame_idx << ", row: " << i << ", label: " << label.str() );
                label_file << label.str() << "\n";
            }
            frame_idx++;
        }
        rate.sleep();
    }

    /* write to file.*/
    label_file.close();
    return 0;

}
