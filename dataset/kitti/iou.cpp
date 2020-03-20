
#include "iou.h"
#include "Eigen/src/Geometry/AngleAxis.h"


#include <cmath>
#include <vector>
#include <map>
#include <iostream>

#include <Eigen/Core>
#include <glog/logging.h>
//#include <jsk_recognition_msgs/BoundingBox.h>
//#include <jsk_recognition_msgs/BoundingBoxArray.h>
//IOU computation.. 
//maybe matrix cooperation can use eigen

struct MyCompareEigenVector2d
{//Overide of operator < () for using Eigen::Vector2d as key in map.
    bool operator() (const Eigen::Vector2d & a, const Eigen::Vector2d & b) const 
    {
        if (a.x() != b.x()) return a.x() < b.x();
        else return a.y() < b.y();
    }
};

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

Rectangle2D::Rectangle2D(Eigen::MatrixXd crns) 
{ 
    corners = crns; 
    Eigen::MatrixXd ecorners(2, 4);
    //corners: (A, B, C, D), ecorners: (B, C, D, A), ecorners - corners: (AB, BC, CD, DA)
    ecorners.block(0, 0, 2, 3) = corners.block(0, 1, 2, 3);
    ecorners.rightCols(1) = corners.leftCols(1);
    edges = ecorners - corners;        
}
bool Rectangle2D::isInsidePoint(const Eigen::Vector2d & p) 
{
    Eigen::Vector2d AP, BP, CP, DP;
    AP = p - corners.col(0);
    BP = p - corners.col(1);
    CP = p - corners.col(2);
    DP = p - corners.col(3);

    return (eigen_vector2d_cross(edges.col(0), AP) * eigen_vector2d_cross(edges.col(2), CP) > 0) 
            && (eigen_vector2d_cross(edges.col(1), BP) * eigen_vector2d_cross(edges.col(3), DP) > 0);
}
bool Rectangle2D::isEndPoint(const Eigen::Vector2d & p)
{
    return corners.col(0) == p || corners.col(1) == p || corners.col(2) == p || corners.col(3) == p;
}
double Rectangle2D::getArea() 
{
    return std::abs(eigen_vector2d_cross(edges.col(0), edges.col(1)));
}


double Rectangle2D::getIntersectArea(Rectangle2D & rec)
{
    std::vector<Eigen::Vector2d> vertexes;
    std::map<Eigen::Vector2d, char, MyCompareEigenVector2d> vertexes_exist;
    //std::cout << "[getIntersectArea] irec.corners: " << this->corners << std::endl;
    //std::cout << "[getIntersectArea] jrec.corners: " << rec.corners << std::endl;
    int last_j = 0;
    for (int i = 0; i < 4; ++i)
    {
        //is ith corner inside of rec? if yes, add to vertexes.
        Eigen::Vector2d ipoint = corners.col(i);
        if (rec.isInsidePoint(ipoint))
        {
            //std::cout << "[getIntersectArea] found a vertex of intersect area, "
            //          <<"because ipoint " << i << " is inside of jrec! " << std::endl; 
            vertexes.push_back(ipoint);
            vertexes_exist[ipoint] = 'y';
        }
        //is ith edge intersect with rec? check each edge of rec.
        Eigen::Vector2d iedge = edges.col(i);
        int iedge_a = i;
        int iedge_b = (i + 1) % 4;
        double i_xmax, i_xmin, i_ymax, i_ymin;
        getMinMaxofTwoCorners(corners.col(iedge_a), corners.col(iedge_b), i_xmax, i_xmin, i_ymax, i_ymin);
        for (int j = last_j, n = 0; n < 4; ++j, ++n)
        {
            j = j % 4;
            //is jth edge intersect with ith edge?
            Eigen::Vector2d jedge = rec.edges.col(j);
            int jedge_a = j;
            int jedge_b = (j + 1) % 4;
            double j_xmax, j_xmin, j_ymax, j_ymin;
            getMinMaxofTwoCorners(rec.corners.col(jedge_a), rec.corners.col(jedge_b), j_xmax, j_xmin, j_ymax, j_ymin);
            if (j_xmin > i_xmax || j_xmax < i_xmin || j_ymin > i_ymax || j_ymax < i_ymin )
            {//jth edge is outside of ith edge.
                //std::cout << "[getIntersectArea] jedge " << j << " is outside of iedge " << i << std::endl;
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
                    std::cerr << "[iou.cpp, getIntersectArea] intersect edge shouldnt be parallel!" << std::endl;
                    //continue;
                }
                double t = eigen_vector2d_cross(ia_ja, jedge) / eigen_vector2d_cross(iedge, jedge);
                Eigen::Vector2d intersect_point = ipoint + t * iedge;
                if (vertexes_exist.find(intersect_point) != vertexes_exist.end())
                    continue;
                //std::cout << "[getIntersectArea] found a vertex of intersect area, " 
                //  << "because iedge " << i <<" is intersect with jedge " << j << " by point " << intersect_point << std::endl;
                last_j = j;
                vertexes.push_back(intersect_point); //? how to deal with repeat ones?                                  
                vertexes_exist[intersect_point] = 'y';
                //next point might be on iedge_b inside point
                Eigen::Vector2d next_point = rec.corners.col(jedge_b);
                if (intersect_point != next_point && this->isInsidePoint(next_point))
                {
                    if (vertexes_exist.find(next_point) != vertexes_exist.end())
                    {
                        std::cerr << "[iou.cpp, getIntersectArea] still got repeat vertex?" << std::endl;
                    }
                    vertexes.push_back(next_point);
                    vertexes_exist[next_point] = 'y';
                }
            }
        }
    }
    if (vertexes.size() == 0)
    {
        //Maybe the whole rec is inside of this rec, or they aren't intersect.
        for (int j = 0; j < 4; ++j)
        {
            if (this->isInsidePoint(rec.corners.col(j)))
            {
                vertexes.push_back(rec.corners.col(j));
                //std::cout << "[getIntersectArea] found intersect vertexes because jpoint " << j <<  " is inside of irec!" << std::endl;
            }
        }
    }
    /*
    std::cout << "[getIntersectArea] output of all vertexes: " << std::endl;
    for (auto i = 0; i < vertexes.size(); ++i)
    {
        //output vertexes
        std::cout << i << ": \n" << vertexes[i] << std::endl;
    }
    */
    if (vertexes.size() < 3)
    {//no intersection
        return 0.0;
    }
    //else
    Eigen::Vector2d pa, pb, pc;
    pa = vertexes[0];
    double area = 0;
    for (auto i = 1; i < vertexes.size() - 1; ++i)
    {
        pb = vertexes[i];
        pc = vertexes[i + 1];
        double triangle_area = std::abs(eigen_vector2d_cross(pb-pa, pc-pa) ) / 2;
        //std::cout << "[getIntersectArea]triangle area: " << triangle_area << " for (0, " << i << ", " << i+1 << " )" << std::endl;
        area += triangle_area;
    }
    return area;
    
}



double BoxIoUBev(const Eigen::Matrix<double, 7, 1> & ibox3d, const Eigen::Matrix<double, 7, 1> & jbox3d, std::string coordinate)
{
    
    //LOG(INFO) << "start get Box IoU at BEV";
    //icorners: 2*4, jcorners: 2*4, four 2D box corner points in clockwise.
    double iou;
    Eigen::MatrixXd corners3d = convertBox3Dto8corners(ibox3d, coordinate);
    Rectangle2D irec(corners3d.topLeftCorner(2, 4));
    corners3d = convertBox3Dto8corners(jbox3d, coordinate);
    Rectangle2D jrec(corners3d.topLeftCorner(2, 4));
    std::cout << "corners of irec: \n" << irec.corners << std::endl; 
    std::cout << "corners of jrec: \n" << jrec.corners << std::endl; 
        
    double sum = irec.getArea() + jrec.getArea();
    double inter = irec.getIntersectArea(jrec);
    std::cout << "[BoxIoUBev]union: " << sum <<" , inter: " << inter << std::endl;
    iou = inter / (sum - inter);
    std::cout << "[BoxIoUBev]iou: " << iou << std::endl;
    if (iou > 1.0)
        std::cerr << "[iou.cpp BoxIoUBev] invalid value of iou: "<< iou << "(>1.0.)! maybe because of wrong intersection area! " << std::endl;
    
    return iou;
}

double CalculateIoU3d(const Eigen::Matrix<double, 7, 1> & ibox3d, const Eigen::Matrix<double, 7, 1> & jbox3d, std::string coordinate)
{
    //box3d: (x, y, z, l, w, h, yaw)
    double ibox3d_h = ibox3d(5);
    double jbox3d_h = jbox3d(5);
    Eigen::MatrixXd ibox3d_corners = convertBox3Dto8corners(ibox3d, coordinate);
    Rectangle2D irec(ibox3d_corners.topLeftCorner(2, 4));
    double ivol = irec.getArea() * ibox3d_h;
    Eigen::MatrixXd jbox3d_corners = Box3dToCorners(jbox3d, coordinate);
    Rectangle2D jrec(jbox3d_corners.topLeftCorner(2, 4));
    double jvol = jrec.getArea() * jbox3d_h;
    double inter_area = irec.getIntersectArea(jrec);
    double inter_h = ibox3d_h < jbox3d_h ? ibox3d_h : jbox3d_h;
    double inter_vol = inter_area * inter_h;
    double iou = inter_vol / (ivol + jvol - inter_vol);
    return iou;
}

 
