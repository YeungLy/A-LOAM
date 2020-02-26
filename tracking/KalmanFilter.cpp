/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : F(F), H(H), Q(Q), R(R), P0(P),
    nz(H.rows()), nx(F.rows()), dt(dt), initialized(false),
    I(nx, nx), x_hat(nx), x_hat_new(nx)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

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
  x_hat_new = F * x_hat;
  P = F*P*F.transpose() + Q;
  
}

void KalmanFilter::update(const Eigen::VectorXd& z) {

  //update
  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
  x_hat_new += K * (z - H*x_hat_new);
  P = (I - K*H)*P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd F) {

  this->F = F;
  this->dt = dt;
  update(z);
}
