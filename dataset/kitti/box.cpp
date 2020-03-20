#include "box.h"
#include "box_utils.h"
#include "rectangle.h"

using namespace kitti;

Eigen::Matrix<double, 3, 8> Box3D::center_to_corners()
{
    //box3d: (x,y,z,l,w,h,yaw)
    //8corners: (x,y,z), 3*8
    //Eigen::MatrixXd corners(3, 8);
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
Eigen::Matrix<double, 2, 8> Box3D::project_to_image(const Eigen::Matrix<double, 3, 4> & P)
{        
    assert(coordinate == "camera")
    //P: projection matrix 
    //homogenous coordinate
    Eigen::Matrix<double, 4, 8> box3d_corners_homo = Eigen::MatrixXd::Ones(4, 8);
    box3d_corners_homo.topLeftCorner(3, 8) = this->center_to_corners();
    Eigen::Matrix<double, 3, 8> box3d_corners_projected_homo = P * box3d_corners_homo;
    Eigen::Array<double, 2, 8> arr = box3d_corners_projected_homo.array().topRows(2);
    arr.rowwise() /= box3d_corners_projected_homo.array().row(2);
    return arr.matrix();
    //std::cout << "[projectBoxtoImage] box3d_corners: \n" << box3d_corners << ", projectd 2d: \n" << projected_box3d << std::endl;
}

Box2D Box3D::get_box2d(const Eigen::Matrix<double, 3, 4> & P)
{        
    Eigen::Matrix<double, 2, 8> box3d_corners_projected = this->project_to_image(P);
    double xmin = box3d_corners_projected.row(0).minCoeff();
    double ymin = box3d_corners_projected.row(1).minCoeff();
    double xmax = box3d_corners_projected.row(0).maxCoeff();
    double ymax = box3d_corners_projected.row(1).maxCoeff();
    return Box2D(xmin, ymin, xmax, ymax);
}
Eigen::Matrix<double, 2, 4> Box3D::bev_corners()
{
    Eigen::Matrix<double, 3, 8> corners3d = this->center_to_corners();
    Eigen::Matrix<double, 2, 4> corners_bev;
    
    if (coordinate == "velodyne")
    {
        //x, y
        corners_bev = corner3d.topLeftCorners(2, 4);
    }
    else if (coordinate == "camera")
    {
        //x, z
        corners_bev.row(0) = corners3d.topLeftCorners(1, 4);
        corners_bev.row(1) = corners3d.bottomLeftCorners(1, 4);
    }
    return corners_bev;
}
double Box3D::iou(const Box3D & j)
{
    //intersection
    double inter_h = h < j.h ? h :j.h;
    //bev intersect area
    Rectangle irec(this->bev_corners());
    Rectangle jrec(j->bev_corners());
    double inter_area = irec.getIntersectArea(jrec);
    double intersect = inter_h * inter_area;
    //union
    double sum = this->volumn() + j.volumn();
    assert(sum > intersect);
    double iou = intersect / (sum - intersect);
    assert(iou <= 1);
    return iou;
}

Box3D Box3D::convertCoordinate(std::string target_coordinate)
{
    if (target_coordinate == this->coordinate)
        return *this;
    Box3D box;
    
}

double Box2D::iou(const Box2D & j)
{
    double left = this->xmin > j.xmin ? this->xmin : j.xmin;
    double top = this->ymin > j.ymin ? this->ymin : j.ymin;
    double right = this->xmax < j.xmax ? this->xmax : j.xmax;
    double bottom = this->ymax > j.ymax ? this->ymax : j.ymax;
    if (left < right && top < bottom)
    {
        double inter = (right - left) * (bottom - top);
        double sum = i.area() + j.area();
        assert(sum - inter > 0);
        return inter / (sum - inter);
    }
}


