// statement

#ifndef TRACKLET_H
#define TRACKLET_H

#include "KalmanFilter.h"
#include "Hungarian.h"

#include <vector>

struct DetectedBox{
    double x_, y_, z_;
    double l_, w_, h_;
    double yaw_;
};
        

class Tracklet
{
    public:

    Tracklet();
    Tracklet(const DetectedBox & det, const int & start_frame, const uint32_t & id);
    
    std::vector<DetectedBox> history_;   // (x, y, z, l, w, h, theta, )
    
    //first occur frame index
    int start_frame_;   
    //number of appear frames
    int duration_;      
    // is this tracklet initialized or potential tracklet?
    bool is_initialized_ = false; 
    //keep appearing frame counts
    uint32_t hit_count_ = 0;   
    //keep disappearing frame counts
    uint32_t miss_count_ = 0;  

    uint32_t id_;

    std::vector<double> params_; //(vx, vy, vz) ?
    KalmanFilter kf_;
    
};

class TrackletManager
{

    public:
  
    //association DetectedBoxs at current frame to existed tracklets
   //unmatched : assign the bool value to unmatch
    void UpdateTracklet(std::vector<DetectedBox> curr_dets);
    void MatchBoxToTracklet(std::vector<DetectedBox> curr_dets);
    void MatchBoxToBox(std::vector<DetectedBox> curr_dets);
    void AddTracklet(DetectedBox first_box, int start_frame);

    void DelTracklet(uint32_t target_id);
    
    //member variable 
    std::vector<Tracklet> tracklets_;
    std::vector<Tracklet> potential_tracklets_;
    uint32_t count_;
};


#endif