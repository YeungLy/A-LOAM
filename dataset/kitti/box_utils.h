#ifndef KITTI_BOX_UTILS
#define KITTI_BOX_UTILS

#include <iostream>
#include <Eigen/Dense>
/* 
 * utils for box 
 */

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

Eigen::Matrix<double, 3, 8> convertBox3Dto8corners(const Eigen::Matrix<double, 7, 1> & box3d, std::string coordinate) 
{
    //box3d: (x,y,z,l,w,h,yaw)
    //8corners: (x,y,z), 3*8
    //Eigen::MatrixXd corners(3, 8);
    double l = box3d(3);
    double w = box3d(4);
    double h = box3d(5);
    Eigen::Matrix<double, 3, 8> corners;
    Eigen::Vector3d euler;
    if (coordinate == "velodyne")
    {
        // x: forward, y: right, z: up, rz is zero when object facing toward (just as x-positive).
        corners << l / 2., -l / 2., -l / 2., l / 2., l / 2., -l / 2., -l / 2., l / 2.,
                    -w / 2., -w / 2., w /2., w / 2., -w / 2., -w / 2., w /2., w / 2.,
                    0, 0, 0, 0, h, h, h, h;
        euler << 0.0, 0.0, box3d(6);
    }
    else if (coordinate == "camera")
    {
        // x: left, y: down, z: forward, ry is zero when object facing left(just as x-positive)
        corners << l / 2., l / 2., -l / 2., -l / 2., l / 2., l / 2., -l / 2., -l / 2.,
                    0, 0, 0, 0, -h, -h, -h, -h,
                    w / 2., -w / 2., -w /2., w / 2., w / 2., -w / 2., -w /2., w / 2.;
         
        euler << 0.0, box3d(6), 0.0;
    }
    else
    {
        std::cerr << "Invalid coordinate string : " << coordinate << ", should be 'velodyne' or 'camera' " << std::endl;
        return corners;
    }

    Eigen::Quaterniond q = euler_to_quaternion(euler);
    Eigen::Vector3d t(box3d(0), box3d(1), box3d(2));
    corners = q.toRotationMatrix() * corners;
    corners.colwise() += t;
    return corners;
}

void projectBoxtoImage(const Eigen::Matrix<double, 3, 8> & box3d_corners, const Eigen::Matrix<double, 3, 4> & P, Eigen::Matrix<double, 2, 8> & projected_box3d)
{
    //box3d_corners, 8corners of box3d
    //P: projection matrix 
    //Eigen::Matrix<double, 3, 8> box3d_corners;
    //convertBox3Dto8corners(box3d, box3d_corners);
    //homogenous coordinate
    
    Eigen::Matrix<double, 4, 8> box3d_corners_homo = Eigen::MatrixXd::Ones(4, 8);
    box3d_corners_homo.topLeftCorner(3, 8) = box3d_corners;
    
    Eigen::Matrix<double, 3, 8> box3d_corners_projected = P * box3d_corners_homo;
    Eigen::Array<double, 2, 8> arr = box3d_corners_projected.array().topRows(2);
    arr.rowwise() /= box3d_corners_projected.array().row(2);
    projected_box3d = arr.matrix();
    //std::cout << "[projectBoxtoImage] box3d_corners: \n" << box3d_corners << ", projectd 2d: \n" << projected_box3d << std::endl;
}

#endif
