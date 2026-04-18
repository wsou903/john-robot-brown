#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

// -- Code Prototypes --------------------------------------------------
void speed_change_smooth();
void disable_motors();
void enable_motors();
void stop();
void reverse();
void forward();
void ccw();
void cw();
void strafe_left();
void strafe_right();
void quarter_turn(int cw_ccw_mode);
void move(float x, float y, float z);
void inverse_kinematics(float vel_x, float vel_y, float omega_z, float *ang_vel_ratio);

#endif
