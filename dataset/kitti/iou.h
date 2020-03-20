#ifndef KITTI_IOU_H
#define KITTI_IOU_H

//#include "tracker.h"

#include <cmath>
#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Core>

//IOU computation.. 
//maybe matrix cooperation can use eigen


class Rectangle2D {

    public:
    Rectangle2D(Eigen::MatrixXd crns); 
    bool isInsidePoint(const Eigen::Vector2d & p);
    bool isEndPoint(const Eigen::Vector2d & p);
    double getArea(); 
    double getIntersectArea(Rectangle2D & rec);
    
    Eigen::MatrixXd corners;
    Eigen::MatrixXd edges;


};

double BoxIoUBev(const Eigen::Matrix<double, 7, 1> & ibox3d, const Eigen::Matrix<double, 7, 1> & jbox3d, std::string coordinate = "velodyne");
double CalculateIoU3d(const Eigen::Matrix<double, 7, 1> & ibox3d, const Eigen::Matrix<double, 7, 1> & jbox3d, std::string coordinate = "velodyne");
double CalculateIoU2d(const Eigen::Matrix<double, 7, 1> & ibox3d, const Eigen::Matrix<double, 7, 1> & jbox3d, std::string coordinate = "velodyne");

#endif
