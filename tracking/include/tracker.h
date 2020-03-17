// statement

#ifndef TRACKER_H
#define TRACKER_H

#include <vector>
#include <string>
#include <map>

#include "track.h"
#include "Hungarian.h"


class Tracker
{

    public:
  
    //association DetectedBoxs at current frame to existed Tracks
   //unmatched : assign the bool value to unmatch
    Tracker();
    ~Tracker();
    
    std::vector< std::vector<double> > CreateDistanceMatrix(const std::vector<DetectedBox> & iBoxes, const std::vector<DetectedBox> & jBoxes);
    
    void Update(const std::vector<DetectedBox> & curr_boxes);
    void CheckNewbornObjects(const std::vector<DetectedBox> & curr_boxes);

    void UpdateGT(const std::vector<DetectedBox> & curr_boxes);

    std::map<int, DetectedBox> GetCurrentObjects();


    void AddTrack(const DetectedBox & first_box, const int & start_frame);
    void AddTrack(const DetectedBox & first_box, const int & start_frame, int id);
    bool DelTrack(const int & target_id);

    int GetTrackIndex(const int & target_id);
    void CleanUpTracks(); //delete disappearing Tracks.
    
    //member variable 
    std::vector<Track> tracks_;

    /*
    maybe new Track.. 
        if its age(hit count) equals MIN_HIT_COUNT, then initialize a new Track.. 
        else delete it once it was not matched at current frame.
    */
    std::vector<DetectedBox> newborn_objects_;
    std::vector<int> newborn_objects_age_;

    int tracks_count_ = 0;
    int frame_idx_ = 0;
    
    double MAX_DIST = 0.6; //?
    double MAX_DIST_NEWBORN = 0.7;
    int MAX_MISS_COUNT = 2;
    int MIN_HIT_COUNT = 2;

    HungarianAlgorithm matcher_;
};


//std::vector<DetectedBox> RosMsgToBoxes(jsk_recognition_msgs::BoundingBoxArrayConstPtr boxes_msg);

#endif
