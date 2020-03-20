#include "track.h"
#include <glog/logging.h>

Track::Track(const DetectedBox & det, const int & start_frame, const int & id){
    stage_ = "Init";
    start_frame_ = start_frame;
    id_ = id;
    history_.clear();
    history_.push_back(det);

    //Setting KalmanFilter parameters
    int nx = 10;  //state dimensions
    int nz = 7;   //measurement dimensions
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(nx, nx);
    F.topRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nz, nx);
    H.topLeftCorner(nz, nz) = Eigen::MatrixXd::Identity(nz, nz);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(nx, nx);
    P0.bottomRightCorner(3, 3) *= 1000.;
    P0 *= 10.;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(nz, nz);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(nx, nx);
    Q.topLeftCorner(7, 7) *= 0.01;

    //how to set init speed?
    double vx, vy, vz;
    vz = 0.01;
    // yaw: (-pi, 0) at y-positive, (0, pi) at y-negative
    // yaw: (-pi/2, pi/2) at x-positive, (-pi, -pi/2) and (pi/2, pi) at x-negative
    LOG(INFO) << "[Track::Track] init state, yaw: " << history_[0].yaw << ", sin value: " << sin(history_[0].yaw) << ", cos: " << cos(history_[0].yaw);
    double direction_x = cos(history_[0].yaw) >= 0 ? 1.0 : -1.0;
    double direction_y = sin(history_[0].yaw) <= 0 ? 1.0 : -1.0;
    vx = direction_x * 1.0;
    vy = direction_y * 1.0;
    
    Eigen::VectorXd init_state(nx);
    init_state << history_[0].x, history_[0].y, history_[0].z, 
                  history_[0].l, history_[0].w, history_[0].h, history_[0].yaw,
                  vx, vy, vz;
    kf_.setParameters(0.0, F, H, Q, R, P0);
    kf_.init(start_frame, init_state);
    
}
void Track::predict()
{
    kf_.predcit();
    DetectedBox curr_box = history_.back();
    DetectedBox pred_box;
    pred_box.x = kf_.state()(0);
    pred_box.y = kf_.state()(1);
    pred_box.z = kf_.state()(2);
    pred_box.l = kf_.state()(3);
    pred_box.w = kf_.state()(4);
    pred_box.h = kf_.state()(5);
    pred_box.yaw = kf_.state()(6);
    history_.push_back(pred_box);
    updated_ = false;   //reset to false for next new frame
    LOG(INFO) << "[Track::predict] id: " << id_ <<", velocity: vx=" << kf_.x_hat(7) <<", vy=" << kf_.x_hat(8) << ", vz=" << kf_.x_hat(9) <<"\npredict next frame box : " << pred_box.getPrintString() <<", from current frame box: " << curr_box.getPrintString();
    //LOG(INFO) << "[Track::predict] id: " << id_  << ", kf_.x_hat \n" << kf_.x_hat <<"\nkf_.x_hat\n" << kf_.x_hat;
    //LOG_IF(INFO, id_ == 1) << "[Track::predict] KF: predict next frame box: " << pred_box.getPrintString() <<", from current frame box: " << curr_box.getPrintString(); 
}
void Track::update(const DetectedBox & det)
{
    Eigen::VectorXd measurement(kf_.nz);
    measurement << det.x, det.y, det.z, det.l, det.w, det.h, det.yaw;
    kf_.update(measurement);
    //LOG_IF(INFO, id_ == 1) << "[Track::update] KF: update predicted box by measurment: " << det.getPrintString() << ", predicted box: " << history_.back().getPrintString(); 
    //LOG(INFO) << "[Track::update] KF: update predicted box by measurment: " << det.getPrintString() << ", predicted box: " << history_.back().getPrintString(); 
       
    size_t now = history_.size() - 1;
    history_[now].x = kf_.state()(0);
    history_[now].y = kf_.state()(1);
    history_[now].z = kf_.state()(2);
    history_[now].l = kf_.state()(3);
    history_[now].w = kf_.state()(4);
    history_[now].h = kf_.state()(5);
    history_[now].yaw = kf_.state()(6);
    //LOG_IF(INFO, id_ == 1) << "[Track::update] after KF update: " << history_.back().getPrintString();
    //LOG(INFO) << "[Track::update] after KF update: " << history_.back().getPrintString();
    updated_ = true;

}

DetectedBox Track::getLatestBox() const 
{
    return history_.back();
}
bool Track::isAlive(int current_frame) const
{
    return current_frame >= start_frame_ && current_frame < end_frame_;
}
