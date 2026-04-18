#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"
#include "kalmanfilter.h"

// these return the filtered distance in cm
float getLeftSR();
float getRightSR();
float getLeftLR();
float getRightLR();
float getUSDistance();

void TestIRSensors();

// gyroscope functions
void calibrateGyro();
void GYRO_reading();
float get_rotation_vector_yaw();

#endif