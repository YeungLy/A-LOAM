// statement

#ifndef TRACKLET_H
#define TRACKLET_H

#include "KalmanFilter.h"
#include "Hungarian.h"
#include <vector>
#include <string>
#include <map>

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
        

class Tracklet
{
    public:

    Tracklet();
    Tracklet(const DetectedBox & det, const int & start_frame, const int & id);
    //~Tracklet();

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

class Tracker
{

    public:
  
    //association DetectedBoxs at current frame to existed tracklets
   //unmatched : assign the bool value to unmatch
    Tracker();
    ~Tracker();
    
    std::vector< std::vector<double> > CreateDistanceMatrix(const std::vector<DetectedBox> & iBoxes, const std::vector<DetectedBox> & jBoxes);
    
    void Update(const std::vector<DetectedBox> & curr_boxes);
    void CheckNewbornObjects(const std::vector<DetectedBox> & curr_boxes);

    void UpdateGT(const std::vector<DetectedBox> & curr_boxes);

    std::map<int, DetectedBox> GetCurrentObjects();


    void AddTracklet(const DetectedBox & first_box, const int & start_frame);
    void AddTracklet(const DetectedBox & first_box, const int & start_frame, int id);
    bool DelTracklet(const int & target_id);

    int GetTrackletIndex(const int & target_id);
    void CleanUpTracklets(); //delete disappearing tracklets.
    
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
    int frame_idx_ = 0;
    
    double MAX_DIST = 0.6; //?
    double MAX_DIST_NEWBORN = 0.7;
    int MAX_MISS_COUNT = 2;
    int MIN_HIT_COUNT = 2;

    HungarianAlgorithm matcher_;
};


//std::vector<DetectedBox> RosMsgToBoxes(jsk_recognition_msgs::BoundingBoxArrayConstPtr boxes_msg);

#endif
