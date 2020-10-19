/*! ------------------------------------------------------------------------------------------------------------------
* @file kalman.h
* @brief DecaWave RTLS kalman functions
*
* @attention
* Copyright 2008, 2014 (c) DecaWave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
*/
#ifndef __KALMAN_H
#define __KALMAN_H

#include <iostream>
#include <math.h>

#include "armadillo"

using namespace arma;

class KalmanFilter
{
public:
    // Constructor
    KalmanFilter(void);
    double process(double dt, double location);

    double measNoiseStd;
    double processNoiseStd;
    vec x_prev;
    bool init;

    // VECTOR VARIABLES
    // s.x = state vector estimate.In the input struct, this is the
    vec x;
    // s.z = observation vector
    vec z;
    // MATRIX VARIABLES :
    // s.A = state transition matrix(defaults to identity).
    mat A;
    // s.P = covariance of the state vector estimate.In the input struct,
    mat P;
    // s.Q = process noise covariance(defaults to zero).
    mat Q;
    //s.R = measurement noise covariance(required).
    mat R;
    // s.H = observation matrix(defaults to identity).
    mat H;
};

void Motion_Filter(KalmanFilter *k, double dt, arma::rowvec &in, arma::rowvec &out);

#endif
