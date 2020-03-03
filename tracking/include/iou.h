#ifndef IOU_H
#define IOU_H

#include "tracklet.h"

#include <cmath>
#include <vector>
#include <iostream>

#include <Eigen/Core>

//IOU computation.. 
//maybe matrix cooperation can use eigen


class Rectangle2D {

    public:
    Rectangle2D(const Eigen::MatrixXd & crns); 
    bool isInsidePoint(const Eigen::Vector2d  p);
    
    double isEndPoint(const Eigen::Vector2d & p);
    double getArea(); 
    double getIntersectArea(const Rectangle2D & rec);
    
    Eigen::MatrixXd corners;
    Eigen::MatrixXd edges;


};
//box3d to 8corners
Eigen::MatrixXd Box3dToCorners(const DetectedBox & box)
{
    //8corners: (x,y,z), 3*8
    Eigen::MatrixXd corners(3, 8);
    corners << box.l / 2., -box.l / 2., -box.l / 2., box.l / 2., box.l / 2., -box.l / 2., -box.l / 2., box.l / 2.,
                -box.w / 2., -box.w / 2., box.w /2., box.w / 2., -box.w / 2., -box.w / 2., box.w /2., box.w / 2.,
                0, 0, 0, 0, box.h, box.h, box.h, box.h;

    Eigen::Quaterniond q(std::cos(box.yaw/2.), 0.0, 0.0, std::sin(box.yaw/2.));
    Eigen::Vector3d t(box.x, box.y, box.z);
    Eigen::MatrixXd t_mat(3, 8);
    t_mat << t, t, t, t, t, t, t, t;
    corners = corners * q + t_mat;
    std::cout << "8 3D corners of box:  \n" << corners << std::endl;

    return corners;
}
double BoxIoUBev(const DetectedBox & ibox, const DetectedBox & jbox)
{
    
    //icorners: 2*4, jcorners: 2*4, four 2D box corner points in clockwise.
    double iou;
    Eigen::MatrixXd corners3d = Box3dToCorners(ibox);
    Rectangle2D irec(corners3d.topLeftCorner(2, 4));
    corners3d = Box3dToCorners(jbox);
    Rectangle2D jrec(corners3d.topLeftCorner(2, 4));
    std::cout << "corners of irec: \n" << irec.corners << std::endl; 
    std::cout << "corners of jrec: \n" << jrec.corners << std::endl; 
        
    double sum = irec.getArea() + jrec.getArea();
    double inter = irec.getIntersectArea(jrec);
    std::cout << "[BoxIoUBev]union: " << sum <<" , inter: " << inter << std::endl;
    iou = inter / (sum - inter);
    std::cout << "[BoxIoUBev]iou: " << iou << std::endl;
    
    return iou;
}

#endif
