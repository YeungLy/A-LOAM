/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "tracking/KalmanFilter.h"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : F_(F), H_(H), Q_(Q), R_(R), P0_(P),
    nz_(H.rows()), nx_(F.rows()), dt_(dt), initialized_(false),
    I_(nx_, nx_), x_hat_(nx_), x_hat_new_(nx_)
{
  I_.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat_ = x0;
  P_ = P0_;
  this->t0_ = t0;
  t_ = t0;
  initialized_ = true;
}

void KalmanFilter::init() {
  x_hat_.setZero();
  P_ = P0_;
  t0_ = 0;
  t_ = t0_;
  initialized_ = true;
}

void KalmanFilter::predcit() {

  if(!initialized_)
    throw std::runtime_error("Filter is not initialized!");

  //predict
  x_hat_new_ = F_ * x_hat_;
  P_ = F_*P_*F_.transpose() + Q_;
  
}

void KalmanFilter::update(const Eigen::VectorXd& z) {

  //update
  K_ = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();
  x_hat_new_ += K_ * (z - H_*x_hat_new_);
  P_ = (I_ - K_*H_)*P_;
  x_hat_ = x_hat_new_;

  t_ += dt_;
}

void KalmanFilter::update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd F) {

  this->F_ = F;
  this->dt_ = dt;
  update(z);
}
