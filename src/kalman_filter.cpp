#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  
  cout<<"initial x"<<x_<<endl;
  cout<<"initial P"<<P_<<endl;
  cout<<"initial F"<<F_<<endl;
  cout<<"initial H"<<H_<<endl;
  cout<<"initial R"<<R_<<endl;
  cout<<"initial Q"<<Q_<<endl;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
  
  	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
	float px = x_(0);
  	float py = x_(1);  
  	float vx = x_(2);  
  	float vy = x_(3);  
  
    float range_est = fmax(sqrt(px * px + py * py), 0.0001);
  	float theta_est = atan2(py,px);
    float doppler_est = (px * vx + py * vy)/range_est;
  
  	VectorXd measurement_est = VectorXd(3);
    measurement_est << range_est, theta_est, doppler_est;
	VectorXd y = z - measurement_est;
	
    /* make sure y(1) is within -pi to pi */
    while (y(1) > M_PI || y(1) <-M_PI)
    {
      if (y(1) > M_PI)
      {
        y(1) -= M_PI;
      }
      else
      {
        y(1) += M_PI;
      }
    }
  
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
  
    	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
  
  
}
