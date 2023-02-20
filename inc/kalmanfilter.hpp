#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <iostream>
#include "Eigen/Dense"

class kalmanfilter
{

public:
    kalmanfilter();
    void start();
private:
    
    Eigen::VectorXd Xn1_n; //Predicted state system vector at time step n+1
    Eigen::VectorXd Xn_n; //Estimated system state vector at time step n
    Eigen::VectorXd Un; //Control variable (input to system)
    Eigen::MatrixXd F; //State transition matrix
    Eigen::MatrixXd G; //Control matrix for mapping to state variables
    Eigen::MatrixXd Pn1_n; //Covariance matrix of current state + 1
    Eigen::MatrixXd Pn_n; //Covariance matrix current state
    Eigen::MatrixXd Q; //Proccess noise matrix
    
    void predict();
    void update();
    
};

#endif // KALMANFILTER_H