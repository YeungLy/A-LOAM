// statement

#ifndef TRACKLET_H
#define TRACKLET_H

#include <vector>

class DetectedBox
{
    public:
    
    double x, y, z; 
    double l, w, h;
    double theta;
    DetectedBox();
    DetectedBox(double x, double y, double z, double l, double w, double h, double theta)

class Tracklet
{
    public:

    Tracklet();
    Tracklet(DetectedBox det, int startframe=0);
    
    
    std::vector<DetectedBox> dets;   // (x, y, z, l, w, h, theta, )
    int start_frame;
    int duration;
    std::vector<double> params; //(vx, vy, vz) ?
    
    bool matched;  //default is true.

}

class TrackletManager
{
    public:
  
    //association DetectedBoxs at current frame to existed tracklets
   //unmatched : assign the bool value to unmatch
    void UpdateTracklet(std::vector<DetectedBox> curr_dets);
    void Association(std::vector<DetectedBox> curr_dets);
    //if add or if delete?
    //member variable 
    std::vector<Tracklet> tracklets;
}
