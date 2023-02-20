#include "../inc/kalmanfilter.hpp"

//Xn_n+1 Predicted state system vector at time step n+1
//Xn_n Estimated system state vecotr at time step n
//Un Control variable (input to system)
//F State transition matrix
//G Control matrix for mapping to state variables

kalmanfilter::kalmanfilter()
{
    
}

void kalmanfilter::start()
{
    std::cout << "Does it reach here";
}

void kalmanfilter::predict()
{
    this->Xn_n = (this->F * this->Xn_n) + (this->G * this->Un); //Equation for State Extrapolation
    this->Pn1_n = (this->F * this->Pn_n * this->F.transpose()) + Q; //Equation for Covariance Matrix
}

void kalmanfilter::update()
{

}