#include <iostream>
#include <cmath>
#include <math.h>

#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict(const double& changeInTime_seconds,
                           const double& processNoise_ax,
                           const double& processNoise_ay ) {
    // update transition matrix F for new time
    F_(0,2) = changeInTime_seconds;
    F_(1,3) = changeInTime_seconds;
    
    // update Q, given new time
    updateCovarianceMatrix(changeInTime_seconds, processNoise_ax, processNoise_ay);
    
    // predict the state
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const MatrixXd &H, const VectorXd &z, const Eigen::MatrixXd &R) {
    // update the state by using Kalman Filter equations
    Eigen::VectorXd z_pred = H * x_;
    VectorXd y = z - z_pred;
    
    auto Ht = H.transpose();
    MatrixXd S = (H * P_ * Ht) + R;
    MatrixXd K = (P_ * Ht) * S.inverse();
    
    x_ = x_ + (K * y);
    
    auto I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const Eigen::MatrixXd &R) {

    //assumes z is measurement given in polar coordinates
    auto hx = Tools::ConvertToPolarCoords(x_[0], x_[1], x_[2], x_[3]);
    auto test = Tools::ConvertToCartesianCoords(hx[0], hx[1], hx[2]);
    MatrixXd Hj = Tools::CalculateJacobian(x_);
    
    // update the state by using Kalman Filter equations
    VectorXd y = z - hx;
    
    //normalize phi between -pi and pi
    while(y(1) > M_PI || y(1) < -M_PI){
        if(y(1) > M_PI) {
            y(1) -= 2*M_PI;
        } else if (y(1) < -M_PI){
            y(1) += 2*M_PI;
        }
    }
    
    MatrixXd Hjt = Hj.transpose();
    
    MatrixXd S = Hj * P_ * Hjt + R;
    MatrixXd K = P_ * Hjt * S.inverse();
    
    x_ = x_ + (K * y);
    MatrixXd K_times_H = K * Hj;
    
    auto I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K_times_H) * P_;
    
}

void KalmanFilter::updateCovarianceMatrix(const double& dt,
                                          const double& ax,
                                          const double& ay) {
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    
    Q_ <<
    (dt4/4)*ax, 0,          (dt3/2)*ax, 0,
    0,          (dt4/4)*ay, 0,          (dt3/2)*ay,
    (dt3/2)*ax, 0,          dt2*ax,     0,
    0,          (dt3/2)*ay, 0,          dt2*ay;
    
}



