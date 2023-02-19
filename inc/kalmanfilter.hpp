#ifndef KALMANFILTER_H
#define KALMANFILTER_H

//#include "Eigen/Dense"
#include <iostream>

class kalmanfilter
{

public:
    kalmanfilter();
    void start();
private:
    //Eigen::VectorXd Xn_n; //Estimated system state vecotr at time step n
   // Eigen::VectorXd Un; //Control variable (input to system)
   // Eigen::MatrixXd F; //State transition matrix
   // Eigen::MatrixXd G; //Control matrix for mapping to state variables

    void predict();
    void update();

};

#endif // KALMANFILTER_H