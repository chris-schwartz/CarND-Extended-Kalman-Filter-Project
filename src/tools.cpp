#include <iostream>
#include "tools.h"
#include "exceptions.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // check the validity of the following inputs:
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        throw illegal_argument_exception();
    }
    
    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){
        
        VectorXd residual = estimations[i] - ground_truth[i];
        
        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    
    //calculate the mean
    rmse = rmse / estimations.size();
    
    //calculate the squared root
    rmse = rmse.array().sqrt();
    
    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  //Calculate a Jacobian matrix.

    MatrixXd Hj(3,4);
    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);
    
    //pre-compute a set of terms to avoid repeated calculation
    double c1 = (px*px) + (py*py);
    if(abs(c1) < 1e-5) {
        c1 = 1e-5;
    }
    
    double c2 = sqrt(c1);
    double c3 = (c1*c2);
    
    //check division by zero

    
    //compute the Jacobian matrix
    Hj << (px/c2),                  (py/c2),                0,      0,
		  -(py/c1),                 (px/c1),                0,      0,
		  py*(vx*py - vy*px)/c3,    px*(px*vy - py*vx)/c3,  px/c2,  py/c2;
    
    return Hj;
}

Eigen::VectorXd Tools::ConvertToPolarCoords(double px, double py, double vx, double vy) {
    auto polarCoords = Eigen::VectorXd(3);
    auto rho = sqrt((px*px) + (py*py));
    
    if(abs(px) < 1e-5) {
        px = 1e-5;
    }
    
    if(rho == 0) {
        rho = 1e-5;
    }
    
    auto phi = atan2(py,px);
    auto rho_dot = ((px*vx) + (py*vy)) / rho;

    polarCoords << rho, phi, rho_dot;
    
    return polarCoords;
}

Eigen::VectorXd Tools::ConvertToCartesianCoords(const double& rho, const double& phi, const double& rho_dot) {
    VectorXd cartesianCoords = Eigen::VectorXd(4);
    
    auto px = rho * cos(phi);
    auto py = rho * sin(phi);
    auto vx = rho_dot * cos(phi);
    auto vy = rho_dot * sin(phi);
    
    cartesianCoords << px, py, vx, vy;
    return cartesianCoords;
}
