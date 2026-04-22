#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float mea_e, float est_e, float q)
{
    _err_measure = mea_e;
    _err_estimate = est_e;
    _q = q;
}

float KalmanFilter::updateEstimate(float mea)
{

    _kalman_gain = _err_estimate / (_err_estimate + _err_measure);              // calculate kalman gain
    _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate); // update the current estimate
    _err_estimate = (1 - _kalman_gain) * _err_estimate;
    _err_estimate = + fabsf(_last_estimate - _current_estimate) * _q;
    _last_estimate = _current_estimate; // save the estimate for next time

    return _current_estimate;
}

// The 'fabsf' part helps the filter adapt if the robot moves quickly. err_estimate = + fabsf(_last_estimate - _current_estimate) * _q;