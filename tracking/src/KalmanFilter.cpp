/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {}

void KalmanFilter::setParameters(
    double dt,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
{
    this->F = F;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->P0 = P;
    this->nz = H.rows();
    this->nx = F.rows();
    this->dt = dt;
    this->initialized = false;
    this->I = Eigen::MatrixXd::Identity(nx, nx); 
    this->x_hat = Eigen::VectorXd(nx);
}


void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::predcit() {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  //predict
  x_hat = F * x_hat;
  P = F*P*F.transpose() + Q;
  
}

void KalmanFilter::update(const Eigen::VectorXd& z) {

  //update
  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
  x_hat += K * (z - H*x_hat);
  P = (I - K*H)*P;


  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd F) {

  this->F = F;
  this->dt = dt;
  update(z);
}
