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
    corners << box.l / 2., -box.l / 2., box.l / 2., -box.l / 2., box.l / 2., -box.l / 2., box.l / 2., -box.l / 2.,
              -box.w / 2., -box.w / 2., box.w /2., box.w / 2., -box.w / 2., -box.w / 2., box.w /2., box.w / 2.,
              0, 0, 0, 0, box.h, box.h, box.h, box.h;
    
    Eigen::Quaterniond q{std::cos(box.yaw/2.), 0.0, 0.0, std::sin(box.yaw/2.)};
    Eigen::Vector3d t(box.x, box.y, box.z);
    Eigen::MatrixXd t_mat(3, 8);
    t_mat << t, t, t, t, t, t, t, t;
    corners = corners * q + t_mat;
    return corners;
}

double BoxIoUBev(const Eigen::MatrixXd & icorners, const Eigen::MatrixXd & jcorners)
{
    double iou;

    double iarea = std::sqrt((icorners(0, 0) - icorners(0, 1)*(icorners(0, 0) - icorners(0, 1) * (icorners(1, 0) - icorners(1, 1)*(icorners(1, 0) - icorners(1, 1));
    double jarea = std::sqrt((jcorners(0, 0) - jcorners(0, 1)*(jcorners(0, 0) - jcorners(0, 1) * (jcorners(1, 0) - jcorners(1, 1)*(jcorners(1, 0) - jcorners(1, 1));
    double union = iarea + jarea;
    double inter;
    return iou;
    
}

//8corners to box3d

#endif
