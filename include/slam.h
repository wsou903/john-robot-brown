#ifndef SLAM_H
#define SLAM_H

#include "config.h"
extern HardwareSerial Serial1;
#define BluetoothSerial Serial1
// this gets called continuously
void budget_slam();
void dump_slam_data();

#endif