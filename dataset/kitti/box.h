#ifndef KITTI_BOX_H
#define KITTI_BOX_H

#include <string>
#include <Eigen/Dense>

namespace kitti {

    class Box3D {
        public:
        
        Box3D(): x(0.0), y(0.0), z(0.0), l(0.0), w(0.0), h(0.0), yaw(0.0), coordinate("velodyne") {}
        Box3D(double x, double y, double z, double l, double w, double h, double yaw, std::string coord="velodyne"): x(x), y(y), z(z), l(l), w(w), h(h), yaw(yaw), coordinate(coord) {}

        Eigen::Matrix<double, 3, 8> center_to_corners() const;
        Eigen::Matrix<double, 2, 4> bev_corners() const;
        
        //project to camera
        //Eigen::Matrix<double, 2, 8> project_to_image(const Eigen::Matrix<double, 3, 4> & P);
        //Box2D get_box2d(const Eigen::Matrix<double, 3, 4> & P);
        
        double volumn() const 
        {
            return l*w*h;
        }
        double bev_area() const
        {
            return l*w;
        }
        std::string getPrintString() const 
        { 
            return "x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", z: " + std::to_string(z) 
               + ", l: " + std::to_string(l) + ", w: " + std::to_string(w) + ", h: " + std::to_string(h)
               + ", yaw: " + std::to_string(yaw); 
        }
        //IoU
        double iou(const Box3D & j) const;
        //data
        double x, y, z, l, w, h, yaw;
        double id;
        std::string coordinate;   //default is veloydne
        
        //transformation? position or rotatation or coordinate transform?


    };

    class Box2D {
        public:
        Box2D(): xmin(0.0), ymin(0.0), xmax(0.0), ymax(0.0) {}
        Box2D(double xmin, double ymin, double xmax, double ymax): xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax) {}
        
        //IoU
        double iou(const Box2D & b) const;
        double area() const {
            return (xmax - xmin) * (ymax - ymin);
        }
        double xmin, ymin, xmax, ymax;


    };

}

#endif
