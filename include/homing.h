#ifndef HOMING_H
#define HOMING_H

#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "helpers.h"


extern float robot_heading_global;

void G28();
void AlignWithWall();
void Align_calc(float* array);
void sweep();
int find_min_index(float* array, int size);

#endif