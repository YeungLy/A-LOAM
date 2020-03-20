#ifndef KITTI_UTILS_H
#define KITTI_UTILS_H
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ctime>
#include <iomanip>

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

Eigen::Matrix<double, 4, 4>  loadCalibrationRigid(const::std::string & calib_path)
{
    Eigen::Matrix<double, 4, 4> calib = Eigen::MatrixXd::Identity(4, 4);
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
    return calib;
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
    return time_ms;
}

void loadObjectLabelToVelo(const std::string label_path, const Eigen::Matrix<double, 4, 4> & T_cam2_velo, std::vector<std::vector<double> > &objs)
{
    
    //objs: (x, y, z, l, w, h, yaw), yaw at obj format is start from left(from camera-x positive axis), rz at tracklet format is start from head(from velo-x positive axis),  positive=clock counterwise
    
    std::ifstream label_file(label_path, std::ifstream::in);
    std::string line;
    while (std::getline(label_file, line)) 
    {
        std::stringstream ss(line);
        double element;
        std::vector<double> obj;
        while (ss >> element && obj.size() < 7)
            obj.push_back(element);
        if (obj.size() != 7)
            std::cerr << "Loading label from " << label_path <<" wrong, there should be at least 7 numbers each line (x,y,z,h,w,l,ry).";
        //transform to velo coordinate
        Eigen::Vector4d pos_at_cam;
        pos_at_cam << obj[0], obj[1], obj[2], 1.0;
        Eigen::Vector4d pos_at_velo;
        pos_at_velo = T_cam2_velo.inverse() * pos_at_cam;
        pos_at_velo = pos_at_velo / pos_at_velo(3);
        obj[0] = pos_at_velo(0);
        obj[1] = pos_at_velo(1);
        obj[2] = pos_at_velo(2);
        obj[6] += M_PI/2;       //add pi/2.
        objs.push_back(obj);

    }
}


Eigen::MatrixXd loadCalibrationCamera(std::string calib_name)
{
    Eigen::MatrixXd calib;
    if (calib_name == "R_rect_00") {
        calib.resize(4, 4);
        calib << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0,
            -9.869795e-03, 9.999421e-01, -4.278459e-03, 0,
            7.402527e-03, 4.351614e-03, 9.999631e-01, 0,
            0, 0, 0, 1;
    }
    else if (calib_name == "R_rect_02") {
        calib.resize(4, 4);
        calib << 9.998817e-01, 1.511453e-02, -2.841595e-03, 0,
                 -1.511724e-02, 9.998853e-01, -9.338510e-04, 0,
                 2.827154e-03, 9.766976e-04, 9.999955e-01, 0,
                 0, 0, 0, 1;
    }
    
    else if (calib_name == "P_rect_00") {

         //P_rect_00: 7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
        calib.resize(3, 4);
        calib << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00,
                 0.000000e+00, 7.215377e+02, 1.728540e+02, 0.000000e+00,
                 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00;
    }
    else if (calib_name == "P_rect_02") {

        //P_rect_02: 7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01 0.000000e+00 7.215377e+02 1.728540e+02 2.163791e-01 0.000000e+00 0.000000e+00 1.000000e+00 2.745884e-03
        calib.resize(3, 4);
        calib << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
                 0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
                 0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03;
    }
    return calib;

}

Eigen::Vector3d Point3DfromVelotoCam2(const std::string & calib_path, const Eigen::Vector3d & p_velo)
{
    //normally cam2
    Eigen::Vector3d p_cam;
    
    Eigen::Matrix<double, 4, 4> Tr_cam0_velo = loadCalibrationRigid(calib_path);

    Eigen::Matrix<double, 4, 4> R_rect_00 = loadCalibrationCamera("R_rect_00");
    Eigen::Matrix<double, 4, 4> T_cam0_velo = R_rect_00 * Tr_cam0_velo;
    Eigen::Vector4d p_velo_homo;
    p_velo_homo << p_velo, 1.0;
    Eigen::Vector4d p_cam_homo = T_cam0_velo * p_velo;
}
Eigen::Vector2d projectfromVelotoCam2(const std::string & calib_path, const Eigen::Vector3d & p_velo)
{
    //normally cam2
    Eigen::Vector3d p_cam;
    
    Eigen::Matrix<double, 4, 4> Tr_cam0_velo = loadCalibrationRigid(calib_path);

    Eigen::Matrix<double, 4, 4> R_rect_00 = loadCalibrationCamera("R_rect_00");
    Eigen::Matrix<double, 4, 4> T_cam0_velo = R_rect_00 * Tr_cam0_velo;
    Eigen::Vector4d p_velo_homo;
    p_velo_homo << p_velo, 1.0;
    Eigen::Vector4d p_cam_homo = T_cam0_velo * p_velo;
}
#endif

