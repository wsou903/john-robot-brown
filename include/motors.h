#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

extern bool function_complete;

extern float inherited_angle;


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
void turn_n_degrees(int cw_ccw_mode);
void move(float x, float y, float z);
void inverse_kinematics(float vel_x, float vel_y, float omega_z, float *ang_vel_ratio);
void drive_straight_poc();
void drive_tothis_poc_GV(float distance);
void drive_tothis_poc(float target_US_distance);
void strafe_straight_poc(int direction);
void strafe_thismuch_poc(int direction, float distance);

#endif
