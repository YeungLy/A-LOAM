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

double eigen_vector2d_cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
    return a.x() * b.y() - a.y() * b.x();
}


struct Rectangle2D {

    Eigen::MatrixXd corners;
    Eigen::MatrixXd edges;

    Rectangle2D(const Eigen::MatrixXd & crns) 
    { 
        corners = crns; 
        Eigen::MatrixXd ecorners(2, 4);
        ecorners.leftCols(1) = corners.rightCols(1);
        ecorners.block(0, 1, 2, 3) = corners.topLeftCorners(2, 3);
        edges = ecorners - corners;        
    }
    bool isInsidePoint(const Eigen::Vector2d & p) 
    {
        Eigen::Vector2d AP, BP, CP, DP;
        AP = p - corners.col(0);
        BP = p - corners.col(1);
        CP = p - corners.col(2);
        DP = p - corners.col(3);

        return (eigen_vector2d_cross(edges.col(0), AP) * eigen_vector2d_cross(edges.col(2), CP) > 0) 
                && (eigen_vector2d_cross(edges.col(1), BP) * eigen_vector2d_cross(edges.col(3), DP) > 0);
    }
    double getArea() 
    {
        return std::abs(eigen_vector2d_cross(edges.col(0), edges.col(1)));
    }
};



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



double BoxIoUBev(const DetectedBox & ibox, const DetectedBox & jbox)
{
    
    //icorners: 2*4, jcorners: 2*4, four 2D box corner points in clockwise.
    double iou;
    const Rectangle2D irec(Box3dToCorners(ibox));
    const Rectangle2D jrec(Box3dToCorners(jbox));
    double union = irec.getArea() + jrec.getArea();
    double inter;
    std::vector<Eigen::Vector2d> vertexes;
    //find vertex of inter polygon.
    for (int i = 0; i < 4; ++i)
    {
        //is ith point inside jbox?
        Eigen::Vector2d & ipoint = irec.corners.col(i);
        if (jrec.isInsidePoint(ipoint))
        {
            vertexes.push_back(ipoint);
        }
        //is ith edge intersect with jbox?
        for (int j = 0; j < 4; ++j)
        {
            //is ith edge intersect with jth edge?
            //are they intersect? 
            //what is the intersect point?
            //will there be repeat?
            //if yes, is (j+1) th point inside ibox?
            Eigen::Vector2d & jpoint = jrec.corners.col( (j+1)%4 );
            if (irec.isInsidePoint(jpoint))
            {
                vertexes.push_back(jpoint);
            }
        }
    }
    
    return iou;
    
}

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


#endif
