#ifndef RECTANGLE_H
#define RECTANGLE_H


#include <cmath>
#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Core>

//IOU computation.. 
//maybe matrix cooperation can use eigen


class Rectangle {

    public:
    Rectangle(Eigen::MatrixXd crns); 
    bool isInsidePoint(const Eigen::Vector2d & p);
    bool isEndPoint(const Eigen::Vector2d & p);
    double getArea(); 
    double getIntersectArea(Rectangle & rec);
    
    Eigen::MatrixXd corners;
    Eigen::MatrixXd edges;


};

double BoxIoUBev(const Eigen::Matrix<double, 7, 1> & ibox3d, const Eigen::Matrix<double, 7, 1> & jbox3d, std::string coordinate = "velodyne");
double CalculateIoU3d(const Eigen::Matrix<double, 7, 1> & ibox3d, const Eigen::Matrix<double, 7, 1> & jbox3d, std::string coordinate = "velodyne");
double CalculateIoU2d(const Eigen::Matrix<double, 7, 1> & ibox3d, const Eigen::Matrix<double, 7, 1> & jbox3d, std::string coordinate = "velodyne");

#endif
