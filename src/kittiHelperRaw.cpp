// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

Eigen::Matrix<double, 4, 4> convertOxtsPose(std::vector<double> pose_data, double &scale)
{
    if (scale == 0)
    {
        scale = cos(pose_data[0] * M_PI / 180.0);
    }
    double er = 6378137;
    double lat = pose_data[0];
    double lon = pose_data[1];
    double tx = scale * lon * M_PI * er / 180;
    double ty = scale * er * log( tan((90+lat) * M_PI / 360) );
    double tz = pose_data[2];
    Eigen::Vector3d translation(tx, ty, tz);
    double rx = pose_data[3]; //roll
    double ry = pose_data[4]; //pitch
    double rz = pose_data[5]; //heading
    Eigen::AngleAxisd rollAngle(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rz, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix<double, 4, 4> pose;
    pose.topLeftCorner(3, 3) = rotation_matrix;
    pose.topRightCorner(3, 1) = translation;
    pose.bottomLeftCorner(1, 4) = Eigen::MatrixXd::Zero(1, 4);
    pose(3, 3) = 1;
    return pose;
}

void loadCalibrationRigid(const::std::string & calib_path, Eigen::Matrix<double, 4, 4> & calib)
{
    std::cout << "Loading calib file from: " << calib_path << std::endl;
    std::ifstream calib_file(calib_path, std::ifstream::in);
    std::string line;

    while (std::getline(calib_file, line)) {
        if (line.find("R:") == 0) {
            line = line.substr(3);
            std::stringstream ss(line);
            double value;
            for (int k = 0; k < 3 * 3; k++) {
                if (ss >> value)
                    calib(k / 3, k % 3) = value;
                else
                    break;
            }
        } else if (line.find("T:") == 0) {
            line = line.substr(3);
            std::stringstream ss(line);
            double value;
            for (int k = 0; k < 3; k++) {
                if (ss >> value)
                    calib(k, 3) = value;
                else
                    break;
            }
        }
    }
    calib.bottomRows(1) = Eigen::MatrixXd::Zero(1, 4);
    calib(3, 3) = 1;
}
double convertStrDatetimetoTimestamp(const std::string & s)
{
    std::tm t{};
    std::istringstream ss(s);
    double ms;
    ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S") >> ms;
    if (ss.fail())
    {
        std::cout << "failed to parse time " << std::endl;
        return -1.0;
    }
    std::time_t timestamp = timegm(&t);
    double time_ms = timestamp*1.0 + ms;
    //std::cout << "timestamp: " << timestamp << " ms: " << ms << std::endl;
    //std::cout << " return: " << std::setiosflags(std::ios::showpoint|std::ios::fixed) << std::setprecision(12) << time_ms << std::endl;
    return time_ms;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder, output_bag_file;
    int num_frame = 5;
    n.getParam("num_frame", num_frame);
    n.getParam("dataset_folder", dataset_folder);
    std::cout << "Reading from " << dataset_folder << '\n';
    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);

    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/camera_init";
    odomGT.child_frame_id = "/ground_truth";

    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/camera_init";

    std::string timestamp_path = "velodyne_points/timestamps.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    std::string calib_path = dataset_folder + "../../../2011_09_26_calib/calib_imu_to_velo.txt";
    Eigen::Matrix<double, 4, 4> T_velo_imu = Eigen::MatrixXd::Zero(4, 4);
    loadCalibrationRigid(calib_path, T_velo_imu);
    calib_path = dataset_folder + "../../../2011_09_26_calib/calib_velo_to_cam.txt";
    Eigen::Matrix<double, 4, 4> T_cam0_velo = Eigen::MatrixXd::Zero(4, 4);
    loadCalibrationRigid(calib_path, T_cam0_velo);

    Eigen::Matrix<double, 4, 4> R_rect_00;
    R_rect_00 << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0,
            -9.869795e-03, 9.999421e-01, -4.278459e-03, 0,
            7.402527e-03, 4.351614e-03, 9.999631e-01, 0,
            0, 0, 0, 1;

    T_cam0_velo = R_rect_00 * T_cam0_velo;

    Eigen::Matrix<double, 4, 4> T_cam0_imu = T_cam0_velo * T_velo_imu;
    std::cout << "calib: \nT_velo_imu:\n" << T_velo_imu << "\nT_cam0_velo\n" << T_cam0_velo << "\nT_cam0_imu\n" << T_cam0_imu << std::endl;

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);
    
    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay);

    //set some initial value.
    Eigen::Matrix<double, 4, 4> pose_init_inv;
    double scale_init = 0;
    double time_init = 0.0;
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        std::cout << "process at line num : " << line_num << std::endl;
        if (line_num > num_frame)
            break;
        double timestamp = convertStrDatetimetoTimestamp(line);
        if (line_num == 0)
            time_init = timestamp;
        timestamp -= time_init;
        //std::cout << "processing timestamp: " << std::setiosflags(std::ios::showpoint|std::ios::fixed) << std::setprecision(12) << timestamp << std::endl;

        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder << "image_02/data/" << std::setfill('0') << std::setw(10) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
        right_image_path << dataset_folder << "image_03/data/" << std::setfill('0') << std::setw(10) << line_num << ".png";
        cv::Mat right_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);

        std::stringstream gt_pose_path;
        gt_pose_path << dataset_folder << "oxts/data/" << std::setfill('0') << std::setw(10) << line_num << ".txt";
        std::ifstream gt_pose_file(gt_pose_path.str(), std::ifstream::in);

        std::getline(gt_pose_file, line);
        std::stringstream pose_stream(line);
        std::vector<double> pose_data;
        double value;
        while (pose_stream >> value)
        {
            pose_data.push_back(value);
            if (pose_data.size() == 6)
                break;
        }
        /* oxts pose to velodyne pose. */

        Eigen::Matrix<double, 4, 4> pose = convertOxtsPose(pose_data, scale_init);
        if (line_num == 0)
            pose_init_inv = pose.inverse();
        //convert to relative pose
        pose = pose_init_inv * pose;
        //std::cout << "imu pose:\n" << pose << std::endl;
        //transform from oxts coodinate to velodyne coordinate
        pose = T_velo_imu * (pose * T_velo_imu.inverse());
        //std::cout << "velo pose:\n" << pose << std::endl;
        //transform from oxts to cam0 coordiantes.
        //pose = T_cam0_velo * (pose * T_cam0_velo.inverse());
        //std::cout << "cam0 pose:\n" << pose << std::endl;

        Eigen::Quaterniond q(pose.topLeftCorner<3, 3>());
        q.normalize();
        Eigen::Vector3d t = pose.topRightCorner<3, 1>();
        pose.topLeftCorner(3, 3) = q.toRotationMatrix();
        pose.topRightCorner(3, 1) = t;

        odomGT.header.stamp = ros::Time().fromSec(timestamp);
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);

        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT.publish(pathGT);

        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "velodyne_points/data/"
                        << std::setfill('0') << std::setw(10) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<float> lidar_intensities;
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            lidar_intensities.push_back(lidar_data[i+3]);

            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);

        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
        pub_image_left.publish(image_left_msg);
        pub_image_right.publish(image_right_msg);

        if (to_bag)
        {
            bag_out.write("/image_left", ros::Time::now(), image_left_msg);
            bag_out.write("/image_right", ros::Time::now(), image_right_msg);
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
            bag_out.write("/path_gt", ros::Time::now(), pathGT);
            bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
        }

        line_num++;
        r.sleep();
    }
    if (to_bag)
        bag_out.close();
    std::cout << "Done \n";


    return 0;
}
