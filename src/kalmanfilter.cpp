#include "../inc/kalmanfilter.hpp"

// x - state vector
// P - uncertainty covariance matrix of state x (process covariance)
// z - measurement vector
// R - uncertainty covariance matrix of sensor that produces z (measurement covariance)
// F - update matrix - used to get predicted x - based on time elapsed and assumed dynamic model being tracked
// H - extraction matrix - used to extract the hypothetical measurement if state x is correct and the sensor is perfect
// Q - noise covariance matrix - adds uncertainty to the process covariance
// S - 'innovation' covariance that combines process covariance and measurement covariance
// y - difference between the actual measurement and the predicted measurement
// K - Kalman gain - contains information on how much weight to place on the current prediction and current observed measurement
//   - that will result the final fused updated state vector and process covariance matrix
//   - computed from P (process covariance), H (extraction), R (measurement covariance)

kalmanfilter::kalmanfilter()
{
    
}

void kalmanfilter::start()
{
    std::cout << "Does it reach here";
}

void kalmanfilter::predict()
{
    //this->Xn_n = (this->F * this->Xn_n) + (this->G * this->Un);
   
}

void kalmanfilter::update()
{

}