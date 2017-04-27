#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
    /**
     * Constructor.
     */
    Tools();
    
    /**
     * Destructor.
     */
    virtual ~Tools();
    
    /**
     * A helper method to calculate RMSE.
     */
    static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
    
    /**
     * A helper method to calculate Jacobians.
     */
    static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
    
    /**
     * A helper method which converts catesian coordinates into its corresponding
     * polar coordinates.
     *
     * @param px X position
     * @param py Y position
     * @param vx Velocity in x direction
     * @param vy Velocity in y direction
     *
     * returns a vector with the following 3 values:
     *     1 - rho, radial distance from the origin
     *     2 - phi, bearing angle in radians representing angle from heading
     *     3 - rho dot, rate of change for rho
     */
    static Eigen::VectorXd ConvertToPolarCoords(double px, double py, double vx, double vy);
    
    static Eigen::VectorXd ConvertToCartesianCoords(const double& phi, const double& rho, const double& rho_dot);
    
};


#endif /* TOOLS_H_ */
