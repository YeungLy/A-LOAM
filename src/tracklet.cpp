//trajectory and track manager
#include "tracklet.h"
#include <vector>


Tracklet::Tracklet(const DetectedBox & det, const int & start_frame, const uint32_t & id){
    start_frame_ = start_frame;
    duration_ = 1;
    id_ = id;
    history_.clear();
    history_.push_back(det);
    hit_count_ = 1;
    is_initialized_ = true;

    kf.nx = 10;  //state dimensions
    kf.nz = 7;   //measurement dimensions
    kf.F = Eigen::MatrixXd::Identity(kf.nx, kf.nx);
    kf.H = Eigen::MatrixXd::Zero(kf.nz, kf.nx);
    kf.H.topLeftCorner(kf.nz, kf.nz) = Eigen::MatrixXd::Identity(kf.nz, kf.nz);
    
    //kf.H = Identity shape: (nz, nx)
    
}

TrackletManager::TrackletManager()
{
    max_id = 0;

}

void TrackletManager::UpdateTracklet()
{
    //predict new state for each tracklet to next time.

}

void TrackletManager::MatchBoxToTracklet(std::vector<DetectedBox> curr_boxes)
{
    //KalmanPredict: predict each valid tracklets from t-1 to t.
    //construct std::map<uint32_t, DetectdBox> using the last box of each tracklet at tracklets
    //match by hugiran algorithm
    /*
    if matched:
        KalmanUpdate()
        reset miss_count to zero.
    else
        add 1 to miss_count
        if miss_count is equal to max_miss_count
            delete this tracklet from tracklets.
        


    */
}

void TrackletManager::MatchBoxToBox(std::vector<DetectedBox> curr_dets)
{

//  match by hugiran algorithm
/* 
    if matched, for example: 
        curr_boxes[0] <-> unmatched_boxes[3]
        unmatched_boxes[3] = curr_boxes[0]
        add 1 to detect_count
        if detect_count is equal to min_detect_count
            initialize a new tracklet using curr_box.
    else
        unmatched_boxes.delete(id)

*/

}