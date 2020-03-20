#ifndef TRACK_H
#define TRACK_H
#include <vector>
#include <string>
#include "KalmanFilter.h"
#include "kitti/box.h"

typede kitti::Box3D DetectedBox;
        

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
    bool isAlive(int current_frame) const;


    std::vector<DetectedBox> history_;   // (x, y, z, l, w, h, yaw)
    //stage: [Init, Tracking, Lost, Dead] 
    std::string stage_;
    //first occur frame index
    int start_frame_;   
    int end_frame_;
    //keep disappearing frame counts
    int miss_count_ = 0;  
    bool updated_ = false;

    int id_;
    //std::vector<double> params_; //(vx, vy, vz) ?
    KalmanFilter kf_;
     
};

#endif
