#include <iostream>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() :
			x_ (4),
			P_(4,4),
			F_(4,4),
			R_laser_(2,2),
			R_radar_(3,3),
			H_(2,4),
			Hj_(3,4)
{

}

KalmanFilter::~KalmanFilter() {}


void KalmanFilter::Predict() {

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;

	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * Ht;

	MatrixXd S = H_ * PHt + R_laser_;
	MatrixXd Si = S.inverse();

	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

	/*
	VectorXd z_pred = Hj_ * x_;
	VectorXd y = z - z_pred;
	 */

	double px = x_(0);
	double py = x_(1);
	double vx = x_(2);
	double vy = x_(3);

	double rho = 0;
	double theta = 0;
	double rho_dot = 0;

	rho = sqrt(px*px + py*py);

	if(fabs(px) < 0.001 || fabs(py) < 0.001){
		cout << "px or py is very small.." << endl;
		theta = 0;
		rho_dot = 0;
	} else {
		theta = atan2(py, px);
		rho_dot = (px*vx + py*vy) / rho;
	}

	VectorXd z_pred = VectorXd(3);
	z_pred << rho, theta, rho_dot;

	while ((z_pred(1) - z(1)) > (M_PI/2.0F)){
		z_pred(1) = z_pred(1) - M_PI;
	}
	while ((z(1) - z_pred(1)) > (M_PI/2.0F)){
		z_pred(1) = z_pred(1) + M_PI;
	}

	VectorXd y = z - z_pred;

	MatrixXd Ht = Hj_.transpose();
	MatrixXd PHt = P_ * Ht;

	MatrixXd S = Hj_ * PHt + R_radar_;
	MatrixXd Si = S.inverse();

	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;

}
