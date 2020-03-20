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

Box2D projectBox3DtoBox2D(const Box3D & box3d)
{
    Box2D box2d;
    
}
#endif
