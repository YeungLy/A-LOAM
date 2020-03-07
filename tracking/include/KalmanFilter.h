/**
* Kalman filter implementation using Eigen. Based on the following
* introductory paper:
*
*     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <Eigen/Dense>

#pragma once

class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A=F - System dynamics matrix, state transition matrix, nx*nx
  *   C=H - Output matrix, Measurement function, nz*nx
  *   Q - Process noise covariance, nx*nx
  *   R - Measurement noise covariance,  nz*nz  
  *   P - Estimate error covariance,  nx*nx
  */

  /**
  * Create a blank estimator.
  */
  KalmanFilter();


  void setParameters(
      double dt,
      const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );
  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const Eigen::VectorXd& x0);

 /**
 *  Predict state from past to current time.
 * 
 */
  void predcit();

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXd& z);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd F);

  /**
  * Return the current state and time.
  */
  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };

//private:

  // Matrices for computation
  Eigen::MatrixXd F, H, Q, R, P, K, P0;

  // System dimensions
  int nx, nz;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};

#endif
