#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in,  MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
 
    // predict the state
    x_ = F_* x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_* P_* Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
    // New state
    x_ = x_ + (K * y);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
    
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    
    // If rho == 0, skip the update step to avoid dividing by zero.
    float c1 = sqrt(px * px + py * py);
    float c2 = atan2(py,px);
    float c3 = (px * vx + py * vy)/c1;
    
    //Feed in equations above
    VectorXd z_pred = VectorXd(3);
    z_pred << c1, c2, c3;
    VectorXd y = z - z_pred;
    
    //// Normalize the angle
    while (y(1) > M_PI) {
        y(1) -= 2 * M_PI;
    }
    while (y(1) <- M_PI) {
        y(1) += 2 * M_PI;
    }
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_* Ht * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
    
}
