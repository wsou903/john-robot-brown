#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "config.h"
#include <math.h>

class KalmanFilter
{
public:
    // mea_e = measurement error (sensor jitter or the swing, if sensor reads 19.5 to 20.5, the measurement error is 1)
    // est_e = estimation error (how  much it trust the first reading, not relevant in this filter so it gets set to same as measurement error)
    // q = process noise (usually a small number between 0.001 and 1, how fast your measurement moves)
    KalmanFilter(float mea_e, float est_e, float q);

    float updateEstimate(float mea);

private:
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;
};

#endif