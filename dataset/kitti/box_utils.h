#ifndef KITTI_BOX_UTILS
#define KITTI_BOX_UTILS

#include "box.h"
#include <iostream>
#include <Eigen/Dense>
/* 
 * utils for box 
 */
using namespace kitti;
Eigen::Quaterniond euler_to_quaternion(Eigen::Vector3d euler)
{
    //std::cout << "euler: (rx, ry, rz) \n " << euler << std::endl;
    double rx, ry, rz;
    rx = euler(0);
    ry = euler(1);
    rz = euler(2);
    Eigen::AngleAxisd rxAxis(rx, Eigen::Vector3d::UnitX()); 
    Eigen::AngleAxisd ryAxis(ry, Eigen::Vector3d::UnitY()); 
    Eigen::AngleAxisd rzAxis(rz, Eigen::Vector3d::UnitZ()); 
    Eigen::Quaterniond q;
    q = rxAxis * ryAxis * rzAxis;
    //std::cout << "after transformation, quaternion: (x, y, z, w) \n" << q.coeffs() << std::endl;
    return q;
}

Eigen::Vector3d quaternion_to_euler(Eigen::Quaterniond q)
{
    //no matter what the convention is to construct quaternion from euler, 
    //it can be departed as rx*ry*rz.
    //std::cout << "quaternion: (x, y, z, w) \n" << q.coeffs() << std::endl;
    //eulerAngles, return angles'range: [
    Eigen::Vector3d euler = q.matrix().eulerAngles(0, 1, 2);
    //std::cout << "after transformation, euler: (rx, ry, rz) \n " << euler << std::endl;
    return euler;
}

Box2D projectToImage(const Eigen::Matrix<double, 3, 8> box3d_corners, const Eigen::Matrix<double, 3, 3>& K)
{
    //to image2.
    //Eigen::Matrix<double, 3, 8> box3d_corners = box3d.center_to_corners();
    Eigen::Matrix<double, 3, 8> box3d_corners_projected_homo = K * box3d_corners;
    Eigen::Array<double, 2, 8> tmp = box3d_corners_projected_homo.array().topRows(2);
    tmp.rowwise() /= box3d_corners_projected_homo.array().row(2);
    double xmin = tmp.row(0).minCoeff();
    double ymin = tmp.row(1).minCoeff();
    double xmax = tmp.row(0).maxCoeff();
    double ymax = tmp.row(1).maxCoeff();
    return Box2D(xmin, ymin, xmax, ymax);
    
}
bool transformVeloToCam3D(const Box3D box_velo, const Eigen::Matrix<double, 4, 4> T_cam0_velo, Eigen::Matrix<double, 3, 8> & corners_cam)
{
    //from velo to cam0
    bool visible_at_cam; 
    Eigen::Matrix<double, 3, 8> corners_velo = box_velo.center_to_corners();
    //box3d corners from velo to cam
    Eigen::Matrix<double, 4, 8> corners_velo_homo = Eigen::MatrixXd::Ones(4, 8);
    corners_velo_homo.topLeftCorner(3, 8) = corners_velo;
    corners_velo_homo = T_cam0_velo * corners_velo_homo;
    //only Eigen::Array can do rowwise division. not Eigen::Matrix.
    Eigen::Array<double, 3, 8> tmp = corners_velo_homo.array().topRows(3);
    tmp.rowwise() /= corners_velo_homo.array().row(3);
    //Eigen::Matrix<double, 3, 8> corners_cam = tmp.matrix();
    corners_cam = tmp.matrix();
    //orientation 3d from velo to cam
    /*
    Eigen::Matrix<double, 3, 2> orientation = box_velo.orientation_vector();
    std::cout << "ori in velo: " << orientation << std::endl;
    Eigen::Matrix<double, 4, 2> ori_homo = Eigen::MatrixXd::Ones(4, 2);
    ori_homo.topLeftCorner(3, 2) = orientation;
    ori_homo = T_cam0_velo * ori_homo;
    Eigen::Array<double, 3, 2> tmp_ori = ori_homo.array().topRows(3);
    tmp_ori.rowwise() /= ori_homo.array().row(3);
    orientation = tmp_ori.matrix(); 
    std::cout << "ori in cam: " << orientation << std::endl;
    */
    //check if it is in the front of camera.
    //if (corners_cam.row(2).minCoeff() < 0.5 || orientation.row(2).minCoeff() < 0.5)
    if (corners_cam.row(2).minCoeff() < 0.5)
        visible_at_cam = false;
    else
        visible_at_cam = true;

    return visible_at_cam;
}

#endif
