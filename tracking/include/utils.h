#ifndef UTILS_H
#define UTILS_H

#include "tracklet.h"

#include <cmath>
#include <vector>
#include <iostream>

#include <Eigen/Dense.h>
//#include <jsk_recognition_msgs/BoundingBox.h>
//#include <jsk_recognition_msgs/BoundingBoxArray.h>
//IOU computation.. 
//maybe matrix cooperation can use eigen

double eigen_vector2d_cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
    return a.x() * b.y() - a.y() * b.x();
}

void getMinMaxofTwoCorners(const Eigen::Vector2d & a, const Eigen::Vector2d & b, double & xmax, double & xmin, double & ymax, double & ymin)
{
    xmax = a.x() > b.x() ? a.x() : b.x();
    xmin = a.x() < b.x() ? a.x() : b.x();
    ymax = a.y() > b.y() ? a.y() : b.y();
    ymin = a.y() < b.y() ? a.y() : b.y();
}
class Rectangle2D {

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
    double isEndPoint(const Eigen::Vector2d & p)
    {
        return corners.col(0) == p || corners.col(1) == p || corners.col(2) == p || corners.col(3) == p;
    }
    double getArea() 
    {
        return std::abs(eigen_vector2d_cross(edges.col(0), edges.col(1)));
    }

    double getIntersectArea(const Rectangle2D & rec)
    {
        std::vector<Eigen::Vector2d> vertexes;
        std::cout << "[getIntersectArea] irec.corners: " << this->corners << std::endl;
        std::cout << "[getIntersectArea] jrec.corners: " << rec.corners << std::endl;
        for (int i = 0; i < 4; ++i)
        {
            //is ith corner inside of rec? if yes, add to vertexes.
            Eigen::Vector2d & ipoint = corners.col(i);
            if (rec.isInsidePoint(ipoint))
            {
                std::cout << "[getIntersectArea] found a vertex of intersect area, "
                          <<"because ipoint " << i << " is inside of jrec! " << std::endl; 
                vertexes.push_back(ipoint);
            }
            //is ith edge intersect with rec? check each edge of rec.
            Eigen::Vector2d & iedge = edges.col(i);
            int iedge_a = i;
            int iedge_b = (i + 1) % 4;
            double i_xmax, i_xmin, i_ymax, i_ymin;
            getMinMaxofTwoCorners(corners.col(iedge_a), corners.col(iedge_b), i_xmax, i_xmin, i_ymax, i_ymin);
            for (int j = 0; j < 4; ++j)
            {
                //is jth edge intersect with ith edge?
                Eigen::Vector2d & jedge = rec.edges.col(j);
                int jedge_a = j;
                int jedge_b = (j + 1) % 4;
                double j_xmax, j_xmin, j_ymax, j_ymin;
                getMinMaxofTwoCorners(rec.corners.col(jedge_a), rec.corners.col(jedge_b), j_xmax, j_xmin, j_ymax, j_ymin);
                if (j_xmin > i_xmax || j_xmax < i_xmin || j_ymin > i_ymax || j_ymax < i_ymin )
                {//jth edge is outside of ith edge.
                    continue;
                }     
                Eigen::Vector2d ja_ia = corners.col(iedge_a) - rec.corners.col(jedge_a);
                Eigen::Vector2d ja_ib = corners.col(iedge_b) - rec.corners.col(jedge_a);
                bool i_intersect_j = eigen_vector2d_cross(ja_ia, jedge) * eigen_vector2d_cross(ja_ib, jedge) < 0;
                Eigen::Vector2d ia_ja = rec.corners.col(jedge_a) - corners.col(iedge_a);
                Eigen::Vector2d ia_jb = rec.corners.col(jedge_b) - corners.col(iedge_a);
                bool j_intersect_i = eigen_vector2d_cross(ia_ja, iedge) * eigen_vector2d_cross(ia_jb, iedge) < 0;
                if (i_intersect_j && j_intersect_i)
                {
                    //has intersection
                    if (eigen_vector2d_cross(iedge, jedge) == 0)
                    {
                        std::cerr << "[getIntersectArea] intersect edge shouldnt be parallel!" << std::endl;
                        //continue;
                    }
                    double t = eigen_vector2d_cross(ia_ja, jedge) / eigen_vector2d_cross(iedge, jedge);
                    Eigen::Vector2d intersect_point = ipoint + t * iedge;
                    bool repeat = False;
                    for (auto v = vertexes.begin(); v != vertexes.end(); ++v)
                    {
                        if (*v == intersect_point) { repeat = true; break;}
                    }
                    if (!repeat)
                    {
                        std::cout << "[getIntersectArea] found a vertex of intersect area, " 
                          << "because iedge " << i <<" is intersect with jedge " << j << " by point " << intersect_point << std::endl;
                        vertexes.push_back(intersect_point); //? how to deal with repeat ones?                                  
                        //next point might be on iedge_b inside point
                        Eigen::Vector2d & next_point = rec.corners.col(jedge_b);
                        if (this->isInsidePoint(next_point))
                        {
                            vertexes.push_back(next_point);
                        }
                    }
                }
            }
        }
        std::cout << "[getIntersectArea] output of all vertexes: " << std::endl;
        for (auto i = 0; i < vertexes.size(); ++i)
        {
            //output vertexes
            std::cout << i << ": \n" << vertexes[i] << std::endl;
        }
        // dynamic planning to split polygon
        if (vertexs.size() < 3)
        {//no intersection
            return 0.0;
        }
        //else
        return 0.0;
        
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
    std::cout << "8 3D corners of box:  \n" << corners << std::endl;

    return corners;
}
/*
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
*/
double BoxIoUBev(const DetectedBox & ibox, const DetectedBox & jbox)
{
    
    //icorners: 2*4, jcorners: 2*4, four 2D box corner points in clockwise.
    double iou;
    Eigen::MatrixXd corners3d = Box3dToCorners(ibox);
    const Rectangle2D irec(corners3d.topLeftCorner(2, 4));
    corners3d = Box3dToCorners(jbox);
    const Rectangle2D jrec(corners3d.topLeftCorner(2, 4));
    std::cout << "corners of irec: \n" << irec.corners << std::endl; 
    std::cout << "corners of jrec: \n" << jrec.corners << std::endl; 
        
    double union = irec.getArea() + jrec.getArea();
    double inter = irec.getIntersectArea(jrec);
    std::cout << "[BoxIoUBev]union: " << union <<" , inter: " << inter << std::endl;
    iou = inter / union - inter;
    std::cout << "[BoxIoUBev]iou: " << iou << std::endl;
    
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
