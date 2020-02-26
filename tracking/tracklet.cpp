//trajectory and track manager
#include "tracklet.h"
#include "utils.h"
#include <glog/logging.h>

Tracklet::Tracklet(const DetectedBox & det, const int & start_frame, const uint32_t & id){
    start_frame_ = start_frame;
    duration_ = 1;
    id_ = id;
    history_.clear();
    history_.push_back(det);

    //Setting KalmanFilter parameters
    kf_.nx = 11;  //state dimensions
    kf_.nz = 7;   //measurement dimensions
    kf_.F = Eigen::MatrixXd::Identity(kf_.nx, kf_.nx);
    kf_.H = Eigen::MatrixXd::Zero(kf_.nz, kf_.nx);
    kf_.H.topLeftCorner(kf_.nz, kf_.nz) = Eigen::MatrixXd::Identity(kf_.nz, kf_.nz);
    kf_.P0 = Eigen::MatrixXd::Identity(kf_.nx, kf_.nx);
    kf_.R = Eigen::MatrixXd::Identity(kf_.nz, kf_.nz);
    kf_.Q = Eigen::MatrixXd::Identity(kf_.nx, kf_.nx);

    Eigen::VectorXd init_state(kf_.nx);
    init_state << history_[0].x, history_[0].y, history_[0].z, 
                  history_[0].l, history_[0].w, history_[0].h, history_[0].yaw,
                  0.0, 0.0, 0.0;
    kf_.init(start_frame, init_state);
    
}

void Tracklet::predict()
{
    kf_.predcit();
    DetectedBox pred_box;
    pred_box.x = kf_.x_hat_new(0);
    pred_box.y = kf_.x_hat_new(1);
    pred_box.z = kf_.x_hat_new(2);
    pred_box.l = kf_.x_hat_new(3);
    pred_box.w = kf_.x_hat_new(4);
    pred_box.h = kf_.x_hat_new(5);
    pred_box.yaw = kf_.x_hat_new(6);
    history_.push_back(pred_box);
    matched_ = false;   //reset to false for next new frame
}

void Tracklet::update(const DetectedBox & det)
{
    Eigen::VectorXd measurement(kf_.nz);
    measurement << det.x, det.y, det.z, det.l, det.w, det.h, det.yaw;
    kf_.update(measurement);
    size_t now = history_.size() - 1;
    history_[now].x = kf_.x_hat(0);
    history_[now].y = kf_.x_hat(1);
    history_[now].z = kf_.x_hat(2);
    history_[now].l = kf_.x_hat(3);
    history_[now].w = kf_.x_hat(4);
    history_[now].h = kf_.x_hat(5);
    history_[now].yaw = kf_.x_hat(6);

}

const DetectedBox & Tracklet::GetLatestBox()
{
    return history_[history_.size() - 1];
}

void TrackletManager::AddTracklet(const DetectedBox & first_box, const int & start_frame)
{
    Tracklet track(first_box, start_frame, tracklets_count_);
    tracklets_count_ += 1;
    tracklets_.push_back(track);
}

bool TrackletManager::DelTracklet(const uint32_t & target_id)
{
    bool found = false;
    for (size_t i = 0; i < tracklets_.size(); ++i)
    {
        if (tracklets_[i].id_ == target_id)
        {
            found = true;
            tracklets_.erase(tracklets_.begin() + i);
            LOG(INFO) << "[DelTracklet]Delete tracklet succeeded, id: " << target_id;
            break;
        }
    }
    if (!found)
        LOG(INFO) << "[DelTracklet]Delete tracklet failed, can not find id: " << target_id;
    return found;        
}


std::vector< std::vector<double> > CreateDistanceMatrix(const std::vector<DetectedBox> & iBoxes, const std::vector<DetectedBox> & jBoxes)
{
    return CalculateIoU3d(iBoxes, jBoxes);
}
   

void TrackletManager::Update(const std::vector<DetectedBox> & curr_boxes)
{
    frame_idx++;
    if (tracklets_.size() == 0)
    {
        //still init..
        CheckNewbornObjects(curr_boxes);
        return;
    }

    std::vector<DetectedBox> pred_boxes;
    for (size_t i = 0; i < tracklets_.size(); ++i)
    {
        tracklets_[i].predict();
        pred_boxes.push_back(tracklets_[i].GetLatestBox());
    }
    //calculate distance matrix
    std::vector< std::vector<double> > dist = CreateDistanceMatrix(curr_boxes, pred_boxes);
    std::vector<int> assignment;
    matcher_.Solve(dist, assignment);
    //if n_curr < n_pred, then there must has some unmatched tracklets
    //if n_curr > n_pred, then there must has some unmatched boxes
    //if n_curr == n_pred, there may be some unmatched boxes because of THRESHOLD

    //filter assignment by maximum dist
    std::vector<DetectedBox> curr_unmatched_boxes;

    for (size_t i_curr = 0; i_curr < assignment.size(); ++i_curr)
    { 
        int i_pred = assignment[i_curr];
        if (i_pred == -1)
        {
            LOG(INFO) << "[Update]n_pred" << tracklets_.size() <<", n_curr: " << curr_boxes.size();
            //n_curr > n_pred
            curr_unmatched_boxes.push_back(curr_boxes[i_curr]);
        }
        else if (dist[i_curr][i_pred] >= MAX_DIST)
        {
            //unmatched.
            LOG(INFO) << "[Update]match failed for curr_box id: " << i_curr << " and tracklet id: " << tracklets_[i_pred].id_ <<", distance: " << dist[i_curr][i_pred];
            //add unmatched box
            curr_unmatched_boxes.push_back(curr_boxes[i_curr]);
        }
        else
        {
            //matched
            LOG(INFO) << "[Update]match succeeded for curr_box id: " << i_curr << " and tracklet id: " << tracklets_[i_pred].id_ <<", distance: " << dist[i_curr][i_pred];
            tracklets_[i_pred].update(curr_boxes[i_curr]);
            tracklets_[i_pred].miss_count_ = 0;
            tracklets_[i_pred].matched_ = true;
        }
    }

    //How to delete an element from vector during for loop of this vector?
    for (size_t i = 0; i < tracklets_.size(); ++i)
    {
        if (!tracklets_[i].matched_)
        {//maybe unmatched because of THRESHOLD or n_curr is smaller than n_pred.
            LOG(INFO) << "[Update]tracklet id: " << tracklets_[i].id_ << " is unmatched.";
            tracklets_[i].miss_count_++;
            if (tracklets_[i].miss_count_ >= MAX_MISS_COUNT)
            {
                LOG(INFO) << "[Update]tracklet id: " << tracklets_[i].id_ << " equals to MAX_MISS_COUNT, delete it!";
                DelTracklet(tracklets_[i].id_);
                --i;
            }
        }
    }

    if (curr_unmatched_boxes.size() > 0)
    {
        CheckNewbornObjects(curr_unmatched_boxes);
    }




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

void TrackletManager::CheckNewbornObjects(const std::vector<DetectedBox> & curr_boxes)
{
    if (newborn_objects_.size() == 0)
    {
        newborn_objects_age_.clear();
        for (size_t i = 0; i < curr_boxes.size(); ++i)
        {
            newborn_objects_.push_back(curr_boxes[i]);
            newborn_objects_age_.push_back(1);
        }
        return;
    }

    
    std::vector< std::vector<double> > dist = CreateDistanceMatrix(curr_boxes, newborn_objects_);
    std::vector<int> assignment;
    std::vector<int> updated(newborn_objects_.size());
    matcher_.Solve(dist, assignment);
    for (size_t i_curr = 0; i_curr < assignment.size(); ++i_curr)
    {
        int i_past = assignment[i_curr];
        if (i_past == -1)
        {
            //unmatched, n_curr > n_past, ignore this curr box.
            continue;
        }
        else if (dist[i_curr][i_past] >= MAX_DIST_NEWBORN)
        {
            //unmatched
            //replace old to new.
            LOG(INFO) << "[CheckNewbornObjects]replace past: " << i_past << " to curr: " << i_curr;
            updated[i_past] = 1;
            newborn_objects_[i_past] = curr_boxes[i_curr];
            newborn_objects_age_[i_past] = 1;
        }
        else
        {
            //matched
            LOG(INFO) << "[CheckNewbornObjects]past box and curr box matched! past: " << i_past << " curr: " << i_curr;
            updated[i_past] = 1;
            newborn_objects_[i_past] = curr_boxes[i_curr];
            newborn_objects_age_[i_past]++;
        }
    }

    for (size_t i = 0; i < newborn_objects_.size(); ++i)
    {
        if (updated[i] == 0)
        {
            //unmatched! delete this object
            LOG(INFO) << "[CheckNewbornObjects]unmatched past box " << i << " delete it!";
            newborn_objects_.erase(newborn_objects_.begin() + i);
            newborn_objects_age_.erase(newborn_objects_age_.begin() + i);
            --i;
        }
        else if (newborn_objects_age_[i] == MIN_HIT_COUNT)
        {
            //matched.. and age is qualified.
            LOG(INFO) << "[CheckNewbornObjects]new born objects!" << i << " add it to tracklets!";
            AddTracklet(newborn_objects_[i], frame_idx);
            newborn_objects_.erase(newborn_objects_.begin() + i);
            newborn_objects_age_.erase(newborn_objects_age_.begin() + i);
            --i;
        }
    }
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


std::map<uint32_t, DetectedBox> TrackletManager::GetCurrentObjects()
{
    std::map<uint32_t, DetectedBox> boxes;
    for (size_t i = 0; i < tracklets_.size(); ++i)
    {
        DetectedBox curr_box = tracklets_[i].GetLatestBox();
        uint32_t id = tracklets_[i].id_;
        boxes[id] = curr_box;
    }
    return boxes;
}
