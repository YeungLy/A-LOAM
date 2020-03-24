#include "box.h"
#include "box_utils.h"
#include "polygon_intersect.h"

using namespace kitti;

Eigen::Matrix<double, 3, 8> Box3D::center_to_corners() const
{
    //box3d: (x,y,z,l,w,h,yaw)
    //8corners: (x,y,z), 3*8
    //Eigen::MatrixXd corners(3, 8);
    Eigen::Matrix<double, 3, 8> corners;
    Eigen::Vector3d euler;
    if (coordinate == "velodyne")
    {
        // x: forward, y: right, z: up, rz is zero when object facing toward (just as x-positive).
        corners << l / 2., l / 2., -l / 2., -l / 2., l / 2., l / 2., -l / 2., -l / 2.,
                    w / 2., -w / 2., -w /2., w / 2., w / 2., -w / 2., -w /2., w / 2.,
                    0, 0, 0, 0, h, h, h, h;
        euler << 0.0, 0.0, yaw;
    }
    else if (coordinate == "camera")
    {
        // x: left, y: down, z: forward, ry is zero when object facing left(just as x-positive)
        corners << l / 2., l / 2., -l / 2., -l / 2., l / 2., l / 2., -l / 2., -l / 2.,
                    0, 0, 0, 0, -h, -h, -h, -h,
                    w / 2., -w / 2., -w /2., w / 2., w / 2., -w / 2., -w /2., w / 2.;
         
        euler << 0.0, yaw, 0.0;
    }
    else
    {
        std::cerr << "Invalid coordinate string : " << coordinate << ", should be 'velodyne' or 'camera' " << std::endl;
        return corners;
    }

    Eigen::Quaterniond q = euler_to_quaternion(euler);
    Eigen::Vector3d t(x, y, z);
    corners = q.toRotationMatrix() * corners;
    corners.colwise() += t;
    return corners;
}

Eigen::Matrix<double, 3, 2> Box3D::orientation_vector() const
{
    //box3d: (x,y,z,l,w,h,yaw)
    //8corners: (x,y,z), 3*8
    //Eigen::MatrixXd corners(3, 8);
    Eigen::Matrix<double, 3, 2> orientation;
    orientation <<  0, 0.7 * l,
                    0, 0,
                    0, 0;
    Eigen::Vector3d euler;
    if (coordinate == "velodyne")
    {
        // x: forward, y: right, z: up, rz is zero when object facing toward (just as x-positive).
        euler << 0.0, 0.0, yaw;
    }
    else if (coordinate == "camera")
    {
        // x: left, y: down, z: forward, ry is zero when object facing left(just as x-positive)
        euler << 0.0, yaw, 0.0;
    }
    else
    {
        std::cerr << "Invalid coordinate string : " << coordinate << ", should be 'velodyne' or 'camera' " << std::endl;
        return orientation;
    }

    Eigen::Quaterniond q = euler_to_quaternion(euler);
    Eigen::Vector3d t(x, y, z);
    std::cout << "orinetaion before rot: " << orientation << "\nrot mat:\n" << q.toRotationMatrix() << std::endl;
    orientation = q.toRotationMatrix() * orientation;
    orientation.colwise() += t;
    return orientation;
}
Eigen::Matrix<double, 2, 4> Box3D::bev_corners() const
{
    Eigen::Matrix<double, 3, 8> corners3d = this->center_to_corners();
    Eigen::Matrix<double, 2, 4> corners_bev;
    
    if (coordinate == "velodyne")
    {
        //x, y
        corners_bev = corners3d.topLeftCorner(2, 4);
    }
    else if (coordinate == "camera")
    {
        //x, z
        corners_bev.row(0) = corners3d.topLeftCorner(1, 4);
        corners_bev.row(1) = corners3d.bottomLeftCorner(1, 4);
    }
    return corners_bev;
}
double Box3D::iou(const Box3D & j) const
{
    //intersection
    double inter_h = h < j.h ? h :j.h;
    //bev intersect area
    //std::cout << "irec: " << this->bev_corners() << std::endl; 
    //std::cout << "jrec: " << j.bev_corners() << std::endl; 
    //[TODO]: iou may be still wrong.. how to sort an 
    double inter_area = intersect_area(this->bev_corners(), j.bev_corners());
    //std::cout << "inter: "<< inter_area << std::endl;
    double intersect = inter_h * inter_area;
    //union
    double sum = this->volumn() + j.volumn();
    assert(sum > intersect);
    
    double iou = intersect / (sum - intersect);
    if (!(iou <= 1 && iou >= 0))
    {
        std::cout << "ibox: " << this->getPrintString() << ", jbox: " << j.getPrintString() << ", iou: " << iou << std::endl;
    }
    assert(iou <= 1 && iou >= 0);
    return iou;
}

double Box2D::iou(const Box2D & j) const
{
    double left = this->xmin > j.xmin ? this->xmin : j.xmin;
    double top = this->ymin > j.ymin ? this->ymin : j.ymin;
    double right = this->xmax < j.xmax ? this->xmax : j.xmax;
    double bottom = this->ymax > j.ymax ? this->ymax : j.ymax;
    if (left < right && top < bottom)
    {
        double inter = (right - left) * (bottom - top);
        double sum = this->area() + j.area();
        assert(sum - inter > 0);
        double iou = inter / (sum - inter);
        assert(iou <= 1 && iou >= 0);
        return iou;
    }
}


