//trajectory and track manager
#include "tracker.h"
#include "kitti/iou.h"
#include <iostream>
#include <glog/logging.h>

Tracker::Tracker(int loglevel)
{
    std::cout <<"log level: " << loglevel << std::endl;
    google::InitGoogleLogging("Tracker");
    google::SetStderrLogging(google::GLOG_INFO);
    FLAGS_minloglevel = loglevel;
    FLAGS_log_dir = "/home/ncslab/Project/SLAM/LidarObjectSLAM/catkin_ws/src/LidarObjectSLAM/log";
    LOG(INFO) << "Start logging..";
    
}
Tracker::~Tracker()
{
    LOG(INFO) << "Shutdown logging..";
    google::ShutdownGoogleLogging();

}
void Tracker::AddTrack(const DetectedBox & first_box, const int & start_frame)
{
    LOG(INFO) << "[AddTrack] create new Track at frame: " <<start_frame << ", id=tracks_count_ : " << tracks_count_;
    Track track(first_box, start_frame, tracks_count_);
    track.updated_ = true;  //intialized state.
    tracks_count_ += 1;
    tracks_.push_back(track);
}
void Tracker::AddTrack(const DetectedBox & first_box, const int & start_frame, int id)
{
    LOG(INFO) << "[AddTrack] create new Track at frame: " <<start_frame << ", id: " << id;
    Track track(first_box, start_frame, id);
    tracks_count_ += 1;
    tracks_.push_back(track);
}

bool Tracker::DelTrack(const int & target_id)
{
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        if (tracks_[i].id_ == target_id)
        {
            tracks_.erase(tracks_.begin() + i);
            LOG(INFO) << "[DelTrack]Delete Track succeeded, id: " << target_id;
            return true;
        }
    }
    LOG(INFO) << "[DelTrack]Delete Track failed, can not find id: " << target_id;
    return false;        
}

int Tracker::GetTrackIndex(const int & target_id)
{
    for (int i = 0; i < tracks_.size(); ++i)
    {
        if (tracks_[i].id_ == target_id)
            return i;
    }
    return -1;
}

void Tracker::CleanUpTracks()
{
    //Add miss count to unmatched Tracks, delete those disappearing Tracks. 
    for (int i = 0; i < tracks_.size(); ++i)
    {
        if (!tracks_[i].updated_)
        {//maybe unmatched because of THRESHOLD or n_curr is smaller than n_pred.
            LOG(INFO) << "[CleanUpTracks]Track id: " << tracks_[i].id_ << " is unmatched.";
            tracks_[i].miss_count_++;
            if (tracks_[i].miss_count_ >= MAX_MISS_COUNT)
            {
                tracks_[i].end_frame_ = frame_idx_;
                LOG(INFO) << "[CleanUpTracks]Track id: " << tracks_[i].id_ << " equals to MAX_MISS_COUNT, delete it!";
                dead_tracks_.push_back(tracks_[i]);
                tracks_.erase(tracks_.begin() + i);
                --i;
            }
        }
    }
}

std::vector< std::vector<double> > Tracker::CreateDistanceMatrix(const std::vector<DetectedBox> & iBoxes, const std::vector<DetectedBox> & jBoxes)
{

    //formula: distance(i, j) = 0.6 * dist_iou + 0.25 * dist_center(normalzied) + 0.15 * dist_size(normalized), range of distance value should be [0, 1]
    //[TODO]if dist_iou = 1 (means iou = 0) , shall we still consider other distance terms or we just set the distance to 1?
    std::vector< std::vector<double> > distMat;
    //temporal variable to save dist_center and dist_size for each(i, j)
    Eigen::MatrixXd dist_center_mat(iBoxes.size(), jBoxes.size());
    Eigen::MatrixXd dist_size_mat(iBoxes.size(), jBoxes.size());
    for (size_t i = 0; i < iBoxes.size(); ++i)
    {
        std::vector<double> distRow;
        for (size_t j = 0; j < jBoxes.size(); ++j)
        {   
            double iou = CalculateIoU3d(iBoxes[i].getEigenMatrix(), jBoxes[j].getEigenMatrix());
            double dist_iou, dist_center, dist_size;
            dist_iou = 1 - iou;
            dist_center = std::sqrt(
                    (iBoxes[i].x - jBoxes[j].x)*(iBoxes[i].x - jBoxes[j].x)
                    + (iBoxes[i].y - jBoxes[j].y)*(iBoxes[i].y - jBoxes[j].y) 
                    + (iBoxes[i].z - jBoxes[j].z)*(iBoxes[i].z - jBoxes[j].z) 
                                );
            dist_center_mat(i, j) = dist_center;
            dist_size = std::sqrt(
                    (iBoxes[i].l - jBoxes[j].l)*(iBoxes[i].l - jBoxes[j].l)  
                    + (iBoxes[i].w - jBoxes[j].w)*(iBoxes[i].w - jBoxes[j].w)  
                    + (iBoxes[i].h - jBoxes[j].h)*(iBoxes[i].h - jBoxes[j].h) 
                                );
            dist_size_mat(i, j) = dist_size;
            //double dist = 0.6 * dist_iou;
            distRow.push_back(dist_iou);
        }
        distMat.push_back(distRow);
    }
    //normalize dist_center and dist_size, add normalized value to final output: distMat
    //[TODO] what if only one iBox and one jBox? then max == min, how to normalize it?
    //max-min normalize
    double max_dist_center = dist_center_mat.maxCoeff();
    double min_dist_center = dist_center_mat.minCoeff();
    double max_dist_size = dist_size_mat.maxCoeff();
    double min_dist_size = dist_size_mat.minCoeff();
    //L2 normalize
    double dist_center_norm = dist_center_mat.norm();
    double dist_size_norm = dist_size_mat.norm();
    
    for (size_t i = 0; i < iBoxes.size(); ++i)
    {
        for (size_t j = 0; j < jBoxes.size(); ++j)
        {
            double dist_center = dist_center_norm == 0 ? 0.0 : dist_center_mat(i, j) / dist_center_norm;
            double dist_size = dist_size_norm == 0 ? 0.0 : dist_size_mat(i, j) / dist_size_norm;
            //double dist_center = ( max_dist_center - dist_center_mat(i, j) ) / ( max_dist_center - min_dist_center);
            //double dist_size = ( max_dist_size - dist_size_mat(i, j) ) / ( max_dist_size - min_dist_size);
            
            assert(!isnan(dist_center));
            assert(!isnan(dist_size));

            double dist_iou = distMat[i][j];

            distMat[i][j] = 0.6 * dist_iou + 0.3 * dist_center + 0.1 * dist_size;

            /*
            LOG(INFO) << "[CreateDistanceMatrix]" 
                      << "\ni: " << i << ", box:" << iBoxes[i].getPrintString() 
                      << "\nj: " << j << ", box: " << jBoxes[j].getPrintString() 
                      << "\niou: " << 1 - dist_iou << ", dist_center: " << dist_center << ", dist_size: " << dist_size 
                      << ", dist: " << distMat[i][j];
            */
        }
    }

    return distMat;
}
   

void Tracker::UpdateGT(const std::vector<DetectedBox> & curr_boxes)
{
    //update boxes_gt (has labels, so dont have to do data association)
    LOG(INFO) << "[UpdateGT] Updating Tracker by curr_boxes at frame " << frame_idx_<< " with gt label";
 
    for (int i = 0; i < tracks_.size(); ++i)
    {
        tracks_[i].predict();
    }
    if (curr_boxes.size() == 0)
    {
        LOG(INFO) << "[UpdateGT] No boxes detected at current frame ";
    }
    for (int i = 0; i < curr_boxes.size(); ++i)
    {
        int index = GetTrackIndex(curr_boxes[i].id);
        if (index == -1)
        {
            //new box
            AddTrack(curr_boxes[i], frame_idx_, curr_boxes[i].id);
        }
        else 
        {
            //LOG(INFO) << "[UpdateGT] updating Track: " << tracks_[index].id_ <<", box: " << tracks_[index].getLatestBox().getPrintString() << " using curr box: " << curr_boxes[i].getPrintString();
            tracks_[index].update(curr_boxes[i]);
        }
    }
    CleanUpTracks();
    frame_idx_++;
}
void Tracker::Update(const std::vector<DetectedBox> & curr_boxes)
{
    LOG(INFO) << "[Update] Updating Tracker by curr_boxes at frame " << frame_idx_;

    //predict each Track's state at current frame unless tracks_.size == 0
    //
    if (tracks_.size()== 0)
    {
        LOG(INFO) << "[Update] No track, still initializing";
        if (curr_boxes.size() == 0)
            LOG(INFO) << "[Update] No box detected at current frame ";
        else
            CheckNewbornObjects(curr_boxes);
        frame_idx_++;
        return;
    }
    std::vector<DetectedBox> pred_boxes;
    for (int i = 0; i < tracks_.size(); ++i)
    {
        tracks_[i].predict();
        pred_boxes.push_back(tracks_[i].getLatestBox());
    }
    if (curr_boxes.size() == 0)
    {
        LOG(INFO) << "[Update] No box detected at current frame ";
    }
    else
    {
        //tracks_.size() != 0 && curr_boxes.size() != 0, match them and update Tracks!

        LOG(INFO) << "[Update] Updating " << tracks_.size() << " Track(s) state by " << curr_boxes.size() <<" current_boxes";
        //calculate distance matrix
        LOG(INFO) << "Calculating distance between curr(i) and pred(j):";
        std::vector< std::vector<double> > dist = CreateDistanceMatrix(curr_boxes, pred_boxes);
        std::vector<int> assignment;
        matcher_.Solve(dist, assignment);
        //if n_curr < n_pred, then there must has some unmatched Tracks
        //if n_curr > n_pred, then there must has some unmatched boxes
        //if n_curr == n_pred, there may be some unmatched boxes because of THRESHOLD

        //filter assignment by maximum dist
        std::vector<DetectedBox> curr_unmatched_boxes;
        double dist_threshold = MAX_DIST;
        for (size_t i_curr = 0; i_curr < assignment.size(); ++i_curr)
        { 
            int i_pred = assignment[i_curr];
            if (i_pred == -1)
            {//unmatched i_curr
                LOG(INFO) << "[Update] i_curr " << i_curr << " is missed during matching, because n_pred: " << tracks_.size() <<" < n_curr: " << curr_boxes.size();
                //n_curr > n_pred
                curr_unmatched_boxes.push_back(curr_boxes[i_curr]);
                continue;
            }
            //LOG(INFO) << "[Update] matching result: i_curr " << i_curr <<" to i_pred " << i_pred << ", dist: " << dist[i_curr][i_pred]; 
            //if (!tracks_[i_pred].isStable())
            //    dist_threshold = MAX_DIST_NEWBORN;
            LOG(INFO) << "[Update]dist threshold: " << dist_threshold;
            if (dist[i_curr][i_pred] >= dist_threshold)
            {
                //unmatched.
                LOG(INFO) << "[Update]match failed for curr_box id: " << i_curr << " and Track id: " << tracks_[i_pred].id_ <<", distance: " << dist[i_curr][i_pred];
                //add to curr_unmatched_boxes
                curr_unmatched_boxes.push_back(curr_boxes[i_curr]);
            }
            else
            {
                //matched
                LOG(INFO) << "[Update]match succeeded for curr_box id: " << i_curr << " and Track id: " << tracks_[i_pred].id_ <<", distance: " << dist[i_curr][i_pred];
                tracks_[i_pred].update(curr_boxes[i_curr]);
                tracks_[i_pred].miss_count_ = 0;
            }
        }

        if (curr_unmatched_boxes.size() > 0)
        {
            LOG(INFO) << "[Update]check " << curr_unmatched_boxes.size() <<" curr_unmatched_boxes with history newborn_objcets, see if there are qualified newborn objects";
            CheckNewbornObjects(curr_unmatched_boxes);
        }
    }

    CleanUpTracks();
    frame_idx_++;
}

void Tracker::CheckNewbornObjects(const std::vector<DetectedBox> & curr_boxes)
{
    LOG(INFO) << "[CheckNewbornObjects] Checking newborn objects";
    if (curr_boxes.size() == 0)
    {
        LOG(INFO) << "[CheckNewbornObjects] No box from current frame to check with history.";
        return;
    }
    else if (newborn_objects_.size() == 0)
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
    //if curr_boxes.size() >= newborn_objcets_.size(), the final size of newborn_objects_ <= curr_boxes.size()  
    //else, the final size of newborn_objects_ <= newborn_objcets_.size()
    int max_size = newborn_objects_.size() >= curr_boxes.size() ? newborn_objects_.size() : curr_boxes.size();
    std::vector<int> updated(max_size, 0);
    matcher_.Solve(dist, assignment);

    int del_count = 0;
    int add_count = 0;
    for (size_t i_curr = 0; i_curr < assignment.size(); ++i_curr)
    {
        int i_past = assignment[i_curr];
        if (i_past == -1)
        {
            //unmatched, n_curr > n_past, add this curr box to newborn_objects_
            newborn_objects_.push_back(curr_boxes[i_curr]);
            newborn_objects_age_.push_back(0);
            updated[newborn_objects_.size() - 1] = 1;
            ++add_count;
            continue;
        }
        else if (dist[i_curr][i_past] >= MAX_DIST_NEWBORN)
        {
            //unmatched
            //replace old to new.
            LOG(INFO) << "[CheckNewbornObjects]replace past: " << i_past << " to curr: " << i_curr;
            newborn_objects_[i_past] = curr_boxes[i_curr];
            newborn_objects_age_[i_past] = 1;
            updated[i_past] = 1;
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
        if (updated[i] = 0)
        {
            //wasnt updataed! delete this useless past box.
            LOG(INFO) << "[CheckNewbornObjects]unmatched past box " << i << " delete it!";
            newborn_objects_.erase(newborn_objects_.begin() + i);
            newborn_objects_age_.erase(newborn_objects_age_.begin() + i);
            updated.erase(updated.begin() + i);
            --i;
            ++del_count;
        }
        else if (newborn_objects_age_[i] == MIN_HIT_COUNT)
        {
            //matched.. and age is qualified.
            LOG(INFO) << "[CheckNewbornObjects]new born object found, add it to Tracks!";
            AddTrack(newborn_objects_[i], frame_idx_);
            newborn_objects_.erase(newborn_objects_.begin() + i);
            newborn_objects_age_.erase(newborn_objects_age_.begin() + i);
            updated.erase(updated.begin() + i);
            --i;
            ++del_count;
        }
        //++i;
    }
    LOG(INFO) << "[CheckNewbornObjects]finish checking, delete from newborn objects(qualified or missed): " << del_count 
              << ", add potential newborn objects from curr_boxes: " << add_count 
              <<" , the current number of potential newborn objects: " << newborn_objects_.size();

}
std::map<int, DetectedBox> Tracker::GetCurrentObjects()
{
    std::map<int, DetectedBox> boxes;
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        DetectedBox curr_box = tracks_[i].getLatestBox();
        int id = tracks_[i].id_;
        LOG(INFO) << "[GetCurrentObjects]id: " << id << ", curr box: " << curr_box.getPrintString() << ", at frame: " << frame_idx_;
        boxes[id] = curr_box;
    }
    return boxes;
}
std::vector<Track> Tracker::GetAllTracks()
{
    std::vector<Track> all(dead_tracks_);
    all.insert(all.end(), tracks_.begin(), tracks_.end());
    return all;
}



