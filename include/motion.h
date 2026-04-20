#ifndef MOTION_H
#define MOTION_H

#include "config.h"

  float integral_sum_ir;
  float integral_sum_gyro;
  float integral_sum_us;
  bool function_complete = false;
  static float prev_err_gyro = 0;

  void strafe_straight_poc();

#endif