/*! ------------------------------------------------------------------------------------------------------------------
* @file kalman.cpp
* @brief DecaWave RTLS kalman functions
*
* @attention
* Copyright 2008, 2014 (c) DecaWave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
*/
#include "kalman.h"
#include "logging.h"

KalmanFilter::KalmanFilter(void)
{
    init = false;

    x.set_size(3);
    x.zeros();

    double dt = 0.1;  // 0.1 m/s
    measNoiseStd = 0.075;
    processNoiseStd = 0.2;

    A << 1 << dt << pow(dt, 2) / 2 << endr
        << 0 << 1 << dt << endr
        << 0 << 0 << 1 << endr;
    H << 1 << 0 << 0 << endr;
    Q << 1 << 0 << 0 << endr
        << 0 << 1 << 0 << endr
        << 0 << 0 << 1 << endr;

    P = Q;
    R = 100;

}

double KalmanFilter::process(double dt, double location)
{
    mat K;
    double v_max = 7; //30 m/s
    if (init)
    {
        z << location;

        A(0, 1) = dt;
        A(0, 2) = pow(dt, 2) / 2;
        A(1, 2) = dt;

        Q << 1 << 0 << 0 << endr
            << 0 << 1 << 0 << endr
            << 0 << 0 << 1 << endr;

        x_prev = x;

        // Prediction for state vector and covariance:
        x = A * x;
        x = x_prev;
        float tmp = x(0) - z(0);
        if (abs(tmp) < dt * v_max)
        {
            P = A * P * trans(A) + Q;

            // Compute Kalman gain factor :
            K = P * trans(H) * inv(H * P * trans(H) + R);

            // Correction based on observation :
            x = x + K * (z - H * x);

            // Covariance Update
            P = P - K * H * P;
        }

    }
    else
    {
        x(0) = location;
        init = true;
    }

    return x(0);
}

void Motion_Filter(KalmanFilter* k, double dt, arma::rowvec& in, arma::rowvec& out)
{
    out(0) = k[0].process(dt, in(0));
    out(1) = k[1].process(dt, in(1));
    out(2) = k[2].process(dt, in(2));

    log_writef("Motion Filter KA : dt : %4.15e : Observed Position : %f %f %f : Corrected Position : %f %f %f \n",
        dt, in(0), in(1), in(2), out(0), out(1), out(2));
}