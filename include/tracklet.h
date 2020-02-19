// statement

#ifndef TRACKLET_H
#define TRACKLET_H

#include <vector>

class DetectedBox
{
    public:
    

    DetectedBox();
    DetectedBox(double x, double y, double z, double l, double w, double h, double yaw): 
        x_(x), y_(y), z_(z), l_(l), w_(w), h_(h), yaw_(yaw) {}

    double x_, y_, z_; 
    double l_, w_, h_;
    double yaw_;
    
    bool is_matched_ = false; 
    

};
        

class Tracklet
{
    public:

    Tracklet();
    Tracklet(DetectedBox det, int start_frame): start_frame_(start_frame){
        dets.clear();
        dets_.push_back(det);
    }
    
    std::vector<DetectedBox> dets_;   // (x, y, z, l, w, h, theta, )
    int start_frame_;
    int duration_;
    std::vector<double> params_; //(vx, vy, vz) ?
    
    bool is_matched_;  //default is true.

};

class TrackletManager
{
    public:
  
    //association DetectedBoxs at current frame to existed tracklets
   //unmatched : assign the bool value to unmatch
    void UpdateTracklet(std::vector<DetectedBox> curr_dets);
    void Association(std::vector<DetectedBox> curr_dets);
    void AddTracklet(DetectedBox first_box, int start_frame);
    
    //if add or if delete?
    //member variable 
    std::map<uint32_t, Tracklet> tracklets;
    uint32_t max_id;
}
