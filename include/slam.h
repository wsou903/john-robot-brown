#ifndef SLAM_H
#define SLAM_H

#include "config.h"
#include "helpers.h"

// funny init
void init_slam();

// this gets called continuously
void budget_slam();

#endif