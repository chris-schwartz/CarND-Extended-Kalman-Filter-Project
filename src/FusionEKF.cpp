#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    
    //measurement covariance matrix - laser
    R_laser_ <<
    0.0225, 0,
    0,      0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ <<
    0.09, 0,      0,
    0,    0.0009, 0,
    0,    0,      0.09;
    
    /**
     TODO:
     * Finish initializing the FusionEKF.
     * Set the process and measurement noises
     */
    H_laser_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    if (!is_initialized_) {
        init(measurement_pack);
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    const double PROCESS_NOISE_AX = 9;
    const double PROCESS_NOISE_AY = 9;
    
    long currentTime = measurement_pack.timestamp_;
    double deltaT_seconds = (currentTime - previous_timestamp_) / 1e6;
    previous_timestamp_ = currentTime;
    
    ekf_.Predict(deltaT_seconds, PROCESS_NOISE_AX, PROCESS_NOISE_AY);
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    /**
     TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_);
    } else {
        // Laser updates
        ekf_.Update(H_laser_, measurement_pack.raw_measurements_, R_laser_);
    }
    
    // print the output
    cout << "x_ = " << ekf_.x_ << endl << endl;
    cout << "P_ = " << ekf_.P_ << endl << endl << endl;
}

void FusionEKF::init(const MeasurementPackage &measurement_pack) {
    auto x = VectorXd(4);
    MatrixXd P = Eigen::MatrixXd(4,4);
    
    P <<
    1000,  0,  0, 0,
    0,  1000,  0, 0,
    0,  0,  1000, 0,
    0,  0,  0, 1000;
    
    MatrixXd Q = Eigen::MatrixXd(4,4);
    Q <<
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
    
    MatrixXd F = Eigen::MatrixXd(4,4);
    F <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    auto measurements = measurement_pack.raw_measurements_;
    double px, py, vx, vy;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        auto coords = Tools::ConvertToCartesianCoords(measurements[0], measurements[1], measurements[2]);
        px = coords[0];
        py = coords[1];
        vx = coords[2];
        vx = coords[3];
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        px = measurements[0];
        py = measurements[1];

        vx = 0;
        vy = 0;
    }
    x << px, py, 0, 0;

    ekf_.Init(x, P, F, Q);
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
}

