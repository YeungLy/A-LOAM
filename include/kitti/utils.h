#ifndef KITTI_UTILS_H
#define KITTI_UTILS_H

#include <eigen3/Eigen/Dense>
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
            ROS_ERROR_STREAM("Loading label from " << label_path <<" wrong, there should be at least 7 numbers each line (x,y,z,h,w,l,ry).");
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



#endif

