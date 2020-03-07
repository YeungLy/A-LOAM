//trajectory and track manager
#include "tracker.h"
#include "iou.h"
#include <glog/logging.h>

Tracklet::Tracklet(const DetectedBox & det, const int & start_frame, const int & id){
    start_frame_ = start_frame;
    duration_ = 1;
    id_ = id;
    history_.clear();
    history_.push_back(det);

    //Setting KalmanFilter parameters
    int nx = 10;  //state dimensions
    //nx = 7;  //state dimensions
    int nz = 7;   //measurement dimensions
    //nz = 4;   //measurement dimensions
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(nx, nx);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nz, nx);
    H.topLeftCorner(nz, nz) = Eigen::MatrixXd::Identity(nz, nz);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(nx, nx);
    P0.bottomRightCorner(3, 3) *= 1000.;
    P0 *= 10.;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(nz, nz);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(nx, nx);
    Q.topLeftCorner(7, 7) *= 0.01;

    Eigen::VectorXd init_state(nx);
    init_state << history_[0].x, history_[0].y, history_[0].z, 
                  history_[0].l, history_[0].w, history_[0].h, history_[0].yaw,
                  0.0, 0.0, 0.0;
    //
    /*
    init_state << history_[0].x, history_[0].y, history_[0].z, 
                  history_[0].yaw, 
                  0.0, 0.0, 0.0;
    //               */

    //kf_ = new KalmanFilter(0.0, F, H, Q, R, P0);
    kf_.setParameters(0.0, F, H, Q, R, P0);
   
    kf_.init(start_frame, init_state);
    
}
///*
Tracklet::~Tracklet()
{
    LOG(INFO) << "[~Tracklet] entering destroy function ";
    //delete kf_;
}
//*/

void Tracklet::predict()
{
    kf_.predcit();
    DetectedBox pred_box;
    //Eigen::VectorXd state = kf_.state();
    pred_box.x = kf_.x_hat_new(0);
    pred_box.y = kf_.x_hat_new(1);
    pred_box.z = kf_.x_hat_new(2);
    pred_box.l = kf_.x_hat_new(3);
    pred_box.w = kf_.x_hat_new(4);
    pred_box.h = kf_.x_hat_new(5);
    pred_box.yaw = kf_.x_hat_new(6);
    /*
    pred_box.yaw = kf_.x_hat_new(3);
    pred_box.l = history_[0].l;
    pred_box.w = history_[0].w;
    pred_box.h = history_[0].h;
    */

    history_.push_back(pred_box);
    matched_ = false;   //reset to false for next new frame
    LOG(INFO) << "[Tracklet::predict] predict next frame box by KF: " << pred_box.getPrintString(); 
}

void Tracklet::update(const DetectedBox & det)
{
    Eigen::VectorXd measurement(kf_.nz);
    measurement << det.x, det.y, det.z, det.l, det.w, det.h, det.yaw;
    //measurement << det.x, det.y, det.z, det.yaw;
    kf_.update(measurement);
    size_t now = history_.size() - 1;
    history_[now].x = kf_.x_hat(0);
    history_[now].y = kf_.x_hat(1);
    history_[now].z = kf_.x_hat(2);
    history_[now].l = kf_.x_hat(3);
    history_[now].w = kf_.x_hat(4);
    history_[now].h = kf_.x_hat(5);
    history_[now].yaw = kf_.x_hat(6);
    /*
    history_[now].yaw = kf_.x_hat(3);
    */
    LOG(INFO) << "[Tracklet::update] update predicted box using KF by measurment: " << history_[now].getPrintString(); 

}

const DetectedBox & Tracklet::GetLatestBox()
{
    return history_[history_.size() - 1];
}

Tracker::Tracker()
{

    google::InitGoogleLogging("Tracker");
    google::SetStderrLogging(google::GLOG_INFO);
    FLAGS_log_dir = "/home/ncslab/Project/SLAM/LidarObjectSLAM/catkin_ws/src/LidarObjectSLAM/log";
    LOG(INFO) << "Start logging..";
    
}
Tracker::~Tracker()
{
    LOG(INFO) << "Shutdown logging..";
    google::ShutdownGoogleLogging();

}
void Tracker::AddTracklet(const DetectedBox & first_box, const int & start_frame)
{
    LOG(INFO) << "[AddTracklet] create new tracklet at frame: " <<start_frame;
    Tracklet track(first_box, start_frame, tracklets_count_);
    tracklets_count_ += 1;
    tracklets_.push_back(track);
    LOG(INFO) << "[AddTracklet] finish push to tracklets_, new track pointer: " << &track;
}

bool Tracker::DelTracklet(const int & target_id)
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


std::vector< std::vector<double> > Tracker::CreateDistanceMatrix(const std::vector<DetectedBox> & iBoxes, const std::vector<DetectedBox> & jBoxes)
{
    return CalculateIoU3d(iBoxes, jBoxes);
}
   

void Tracker::Update(const std::vector<DetectedBox> & curr_boxes)
{
    LOG(INFO) << "[Update] Updating Tracker by curr_boxes at frame " << frame_idx_;
    if (curr_boxes.size() == 0)
    {
        LOG(INFO) << "[Update] No boxes detected at current frame ";
        frame_idx_++;
        return;
    }
    if (tracklets_.size() == 0)
    {
        LOG(INFO) << "[Update] Still initializing tracklets";
        //still init..
        CheckNewbornObjects(curr_boxes);
        frame_idx_++;
        return;
    }

    LOG(INFO) << "[Update] updating tracklet state by curr_boxes, tracklets size: " << tracklets_.size() <<", curr boxes size: " << curr_boxes.size();
    std::vector<DetectedBox> pred_boxes;
    for (size_t i = 0; i < tracklets_.size(); ++i)
    {
        tracklets_[i].predict();
        pred_boxes.push_back(tracklets_[i].GetLatestBox());
    }
    for (size_t i = 0; i < curr_boxes.size(); ++i)
    {
        LOG(INFO) << "[Update] curr boxes i: " << i << ", " << curr_boxes[i].getPrintString();
    
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
        {//unmatched i_curr
            LOG(INFO) << "[Update] i_curr " << i_curr << " is missed during matching, because n_pred: " << tracklets_.size() <<" < n_curr: " << curr_boxes.size();
            //n_curr > n_pred
            curr_unmatched_boxes.push_back(curr_boxes[i_curr]);
            continue;
        }
        LOG(INFO) << "[Update] matching result: i_curr " << i_curr <<" to i_pred " << i_pred << ", dist: " << dist[i_curr][i_pred]; 
        if (dist[i_curr][i_pred] >= MAX_DIST)
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
    for (int i = 0; i < tracklets_.size(); ++i)
    {
        if (!tracklets_[i].matched_)
        {//maybe unmatched because of THRESHOLD or n_curr is smaller than n_pred.
            LOG(INFO) << "[Update]tracklet id: " << tracklets_[i].id_ << " is unmatched.";
            tracklets_[i].miss_count_++;
            if (tracklets_[i].miss_count_ >= MAX_MISS_COUNT)
            {
                LOG(INFO) << "[Update]tracklet id: " << tracklets_[i].id_ << " equals to MAX_MISS_COUNT, delete it!";
                tracklets_.erase(tracklets_.begin() + i);
                //DelTracklet(tracklets_[i].id_);
                --i;
            }
        }
    }

    if (curr_unmatched_boxes.size() > 0)
    {
        LOG(INFO) << "[Update]check curr_unmatched_boxes with history newborn_objcets, see if there are qualified newborn objects";
        CheckNewbornObjects(curr_unmatched_boxes);
    }

    frame_idx_++;



    //KalmanPredict: predict each valid tracklets from t-1 to t.
    //construct std::map<int, DetectdBox> using the last box of each tracklet at tracklets
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

void Tracker::CheckNewbornObjects(const std::vector<DetectedBox> & curr_boxes)
{
    LOG(INFO) << "[CheckNewbornObjects] Checking newborn objects";
    if (newborn_objects_.size() == 0)
    {
        newborn_objects_age_.clear();
        for (size_t i = 0; i < curr_boxes.size(); ++i)
        {
            newborn_objects_.push_back(curr_boxes[i]);
            newborn_objects_age_.push_back(1);
        }
        LOG(INFO) << "[CheckNewbornObjects] history newborn_objects is empty, set curr boxes as newborn_objects, now newborn_objects_.size: " << newborn_objects_.size() << ", return. ";
        return;
    }

    LOG(INFO) << "past newborn objects: " << newborn_objects_.size() << ", curr unmatched boxes: " << curr_boxes.size();
    
    std::vector< std::vector<double> > dist = CreateDistanceMatrix(curr_boxes, newborn_objects_);
    std::vector<int> assignment;
    std::vector<int> updated(newborn_objects_.size(), 0);
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
    //use integer instead of size_t
    //   when i = 0, --i and ++i make i keeping zero, but using size_t i = 0, --i will make it a big positive number instead of -1.
    for (int i = 0; i < newborn_objects_.size(); ++i)
    {
        if (updated[i] == 0)
        {
            //unmatched! delete this object
            LOG(INFO) << "[CheckNewbornObjects]unmatched past box " << i << " delete it!";
            newborn_objects_.erase(newborn_objects_.begin() + i);
            newborn_objects_age_.erase(newborn_objects_age_.begin() + i);
            updated.erase(updated.begin() + i);
            --i;
        }
        else if (newborn_objects_age_[i] == MIN_HIT_COUNT)
        {
            //matched.. and age is qualified.
            LOG(INFO) << "[CheckNewbornObjects]new born objects!" << i << " add it to tracklets!";
            AddTracklet(newborn_objects_[i], frame_idx_);
            newborn_objects_.erase(newborn_objects_.begin() + i);
            newborn_objects_age_.erase(newborn_objects_age_.begin() + i);
            updated.erase(updated.begin() + i);
            --i;
        }
        ++i;
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


std::map<int, DetectedBox> Tracker::GetCurrentObjects()
{
    std::map<int, DetectedBox> boxes;
    for (size_t i = 0; i < tracklets_.size(); ++i)
    {
        DetectedBox curr_box = tracklets_[i].GetLatestBox();
        int id = tracklets_[i].id_;
        boxes[id] = curr_box;
    }
    return boxes;
}


