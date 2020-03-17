#ifndef TRACK_H
#define TRACK_H
#include <vector>
#include <string>
#include "KalmanFilter.h"


struct DetectedBox{
    double x, y, z;
    double l, w, h;
    double yaw;
    int id;
    DetectedBox(): x(), y(), z(), l(), w(), h(), yaw(), id() {}
    DetectedBox(double x, double y, double z, double l, double w, double h, double yaw):x(x),y(y),z(z),l(l),w(w),h(h),yaw(yaw), id(-1) {}
    DetectedBox(double x, double y, double z, double l, double w, double h, double yaw, int id):x(x),y(y),z(z),l(l),w(w),h(h),yaw(yaw), id(id) {}
    std::string getPrintString() const 
    { 
        return "x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", z: " + std::to_string(z) 
               + ", l: " + std::to_string(l) + ", w: " + std::to_string(w) + ", h: " + std::to_string(h)
               + ", yaw: " + std::to_string(yaw); 
    }
    
    int getID() const
    {
        return id;
    }

    bool isValid() const 
    {
        return l > 0 && w > 0 && h > 0;
    }

};
        

class Track
{
    public:

    Track();
    Track(const DetectedBox & det, const int & start_frame, const int & id);
    //~Track();

    //predict object at current frame's position
    void predict();
    //update the estimation of object at current frame by the detected box for this object.
    void update(const DetectedBox & det);

    DetectedBox getLatestBox() const; 
    bool isStable() const;


    std::vector<DetectedBox> history_;   // (x, y, z, l, w, h, yaw)
    
    //first occur frame index
    int start_frame_;   
    //number of appear frames
    int age_;      
    //keep disappearing frame counts
    int miss_count_ = 0;  
    bool matched_ = false;

    int id_;
    //std::vector<double> params_; //(vx, vy, vz) ?
    KalmanFilter kf_;
     
};

#endif