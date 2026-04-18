#ifndef FARMING_H
#define FARMING_H

#include "config.h"
#include "sensors.h"
#include "helpers.h"
#include "motors.h"

float Kp;
float Ki;
float Kd;
float ortho_error;
float prev_error;
int baseSpeed;
int maxCorrection;
int num_tramlines;
float laneTargets[20];

void farming_init();

void Farming();

#endif