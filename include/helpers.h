#ifndef HELPERS_H
#define HELPERS_H

#include "config.h"
#include "motors.h"
#include "sensors.h"

int FindMinIndex(float arr[], int n);
int getIndex(float arr[], int n, float desired);
void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();
void AlignWithWall(int lr_sr_mode);

#endif