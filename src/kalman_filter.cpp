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

	cout << "y 0 =" << y << endl;

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_laser_;
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

	/*
	VectorXd z_pred = Hj_ * x_;
	VectorXd y = z - z_pred;
	 */

	double px = x_(0);
	double py = x_(1);
	double vx = x_(2);
	double vy = x_(3);

	double rho = sqrt(px*px + py*py);
	double theta = atan2(py, px);
	double rho_dot = (px*vx + py*vy) / rho;
	VectorXd h = VectorXd(3);
	h << rho, theta, rho_dot;
	VectorXd y = z - h;

	MatrixXd Ht = Hj_.transpose();
	MatrixXd S = Hj_ * P_ * Ht + R_radar_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;

}
