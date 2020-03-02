#ifndef UTILS_H
#define UTILS_H

#include "tracklet.h"

#include <cmath>
#include <vector>

#include <Eigen/Dense.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
//IOU computation.. 
//maybe matrix cooperation can use eigen

std::vector< std::vector<double> > CalculateIoU3d(std::vector<DetectedBox> iBoxes, std::vector<DetectedBox> jBoxes)
{
    std::vector< std::vector<double> > dist;
    for (size_t i = 0; i < iBoxes.size(); ++i)
    {
        std::vector<double> dist_row;
        for (size_t j = 0; j < jBoxes.size(); ++j)
        {
            
            dist_row[j] = 1 - 0.02*i + 0.15*j;
        }
        dist.push_back(dist_row);
    }
    return dist;
}

std::vector<DetectedBox> RosMsgToBoxes(jsk_recognition_msgs::BoundingBoxArrayConstPtr boxes_msg)
{

    std::vector<DetectedBox> boxes;
    for (size_t i = 0; i < boxes_msg->boxes.size(); ++i)
    {
        DetectedBox box;
        box.x = boxes_msg->boxes[i].pose.position.x;
        box.y = boxes_msg->boxes[i].pose.position.y;
        box.z = boxes_msg->boxes[i].pose.position.z;
        //box.yaw = boxes_msg->boxes[i].pose.orientation.
        box.l = boxes_msg->boxes[i].dimensions.x;
        box.w = boxes_msg->boxes[i].dimensions.y;
        box.h = boxes_msg->boxes[i].dimensions.z;

    }
    return boxes;
}

//box3d to 8corners
Eigen::MatrixXd Box3dToCorners(const DetectedBox & box)
{
    //8corners: (x,y,z), 3*8
    Eigen::MatrixXd corners(3, 8);
    corners << box.l / 2., -box.l / 2., -box.l / 2., box.l / 2., box.l / 2., -box.l / 2., -box.l / 2., box.l / 2.,
              -box.w / 2., -box.w / 2., box.w /2., box.w / 2., -box.w / 2., -box.w / 2., box.w /2., box.w / 2.,
              0, 0, 0, 0, box.h, box.h, box.h, box.h;
    
    Eigen::Quaterniond q{std::cos(box.yaw/2.), 0.0, 0.0, std::sin(box.yaw/2.)};
    Eigen::Vector3d t(box.x, box.y, box.z);
    Eigen::MatrixXd t_mat(3, 8);
    t_mat << t, t, t, t, t, t, t, t;
    corners = corners * q + t_mat;
    return corners;
}

double eigen_vector2d_cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
    return a.x() * b.y() - a.y() * b.x();
}
double BoxIoUBev(const Eigen::MatrixXd & icorners, const Eigen::MatrixXd & jcorners)
{
    //icorners: 2*4, jcorners: 2*4, four 2D box corner points in clockwise.
    double iou;


    std::vector<Eigen::Vector2d> iedges(4);
    std::vector<Eigen::Vector2d> jedges(4);
    for (int p = 0; p < 4; ++p)
    {
        iedges.push_back(icorners.col( (p+1)%4 ) - icorners.col(p)); 
        jedges.push_back(jcorners.col( (p+1)%4 ) - jcorners.col(p));      
    }    
    double iarea = std::abs(eigen_vector2d_cross(iedges[0], iedges[1]));
    double jarea = std::abs(eigen_vector2d_cross(jedges[0], jedges[1]));
    double union = iarea + jarea;
    double inter;
    std::vector<Eigen::Vector2d> vertexes;
    //find vertex of inter polygon.
    for (int i = 0; i < 4; ++i)
    {
        //is ith point inside jbox?
        Eigen::Vector2d & ipoint = icorners.col(i);
        if (ipoint(0) >= j_xmin && ipoint(0) <= j_xmax && ipoint(1) >= j_ymin && ipoint(1) <= j_ymax)
        {
            //maybe inside
            //ABxAP
            Eigen::Vector2d AP, BP, CP, DP;
            AP = ipoint - jcorners.col(0);
            BP = ipoint - jcorners.col(1);
            CP = ipoint - jcorners.col(2);
            DP = ipoint - jcorners.col(3);
            if (eigen_vector2d_cross(jedges[0], AP) * eigen_vector2d_cross(jedges[2], CP) > 0) && (eigen_vector2d_cross(jedges[1], BP) * eigen_vector2d_cross(jedges[3], CP) > 0)
            {
                vertexes.push_back(ipoint);
            }
        }
        //is ith edge intersect with jbox?
        for (int j = 0; j < 4; ++j)
        {
            //is ith edge intersect with jth edge?

            //if yes, is (j+1) th point inside ibox?

        }
    }
    
    return iou;
    
}

bool IsInsideRectangle(const Eigen::Vector2d & p, const Eigen::Matrix2d & rec)
{
    bool inside = false;
    double xmax, xmin, ymax, ymin;
    xmax = rec.row(0).maxCoeff();
    xmin = rec.row(0).minCoeff();
    ymax = rec.row(1).maxCoeff();
    ymin = rec.row(1).minCoeff();
    
    if (p.x() >= xmin && p.x() <= xmax && p.y() >= ymin && p.y() <= ymax)
    {

    }

    return inside;
}
//8corners to box3d

#endif
