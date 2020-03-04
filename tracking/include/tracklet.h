// statement

#ifndef TRACKLET_H
#define TRACKLET_H

#include "KalmanFilter.h"
#include "Hungarian.h"

#include <vector>
#include <map>

struct DetectedBox{
    double x, y, z;
    double l, w, h;
    double yaw;
    DetectedBox(): x(), y(), z(), l(), w(), h(), yaw() {}
    DetectedBox(double x, double y, double z, double l, double w, double h, double yaw):x(x),y(y),z(z),l(l),w(w),h(h),yaw(yaw) {}
};
        

class Tracklet
{
    public:

    Tracklet();
    Tracklet(const DetectedBox & det, const int & start_frame, const int & id);

    //predict object at current frame's position
    void predict();
    //update the estimation of object at current frame by the detected box for this object.
    void update(const DetectedBox & det);

    const DetectedBox & GetLatestBox();

    std::vector<DetectedBox> history_;   // (x, y, z, l, w, h, yaw)
    
    //first occur frame index
    int start_frame_;   
    //number of appear frames
    int duration_;      
    //keep disappearing frame counts
    int miss_count_ = 0;  
    bool matched_ = false;

    int id_;
    //std::vector<double> params_; //(vx, vy, vz) ?
    KalmanFilter kf_;
     
};

class TrackletManager
{

    public:
  
    //association DetectedBoxs at current frame to existed tracklets
   //unmatched : assign the bool value to unmatch
    TrackletManager();
    ~TrackletManager();
    
    std::vector< std::vector<double> > CreateDistanceMatrix(const std::vector<DetectedBox> & iBoxes, const std::vector<DetectedBox> & jBoxes);
    
    void Update(const std::vector<DetectedBox> & curr_boxes);
    void CheckNewbornObjects(const std::vector<DetectedBox> & curr_boxes);

    std::map<int, DetectedBox> GetCurrentObjects();

    void AddTracklet(const DetectedBox & first_box, const int & start_frame);
    bool DelTracklet(const int & target_id);
    
    //member variable 
    std::vector<Tracklet> tracklets_;

    /*
    maybe new tracklet.. 
        if its age(hit count) equals MIN_HIT_COUNT, then initialize a new tracklet.. 
        else delete it once it was not matched at current frame.
    */
    std::vector<DetectedBox> newborn_objects_;
    std::vector<int> newborn_objects_age_;

    int tracklets_count_ = 0;
    int frame_idx = 0;
    
    double MAX_DIST = 0.5; //?
    double MAX_DIST_NEWBORN = 0.3;
    int MAX_MISS_COUNT = 2;
    int MIN_HIT_COUNT = 3;

    HungarianAlgorithm matcher_;
};


#endif
