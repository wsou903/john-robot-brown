

#include "motors.h"
#include "helpers.h"
#include "sensors.h"
#include "slam.h"

// Code Starts ----------------------------------------------------------------------------------------------------
int n = 4;
float integral_sum_ir;
float integral_sum_gyro;
float integral_sum_us;

int ir_drive_toggle = 0;

bool function_complete = false; // global flag for function completion (its like an fsm but shit)

// global flags for completions (its like an fsm but shit)

static float prev_err_gyro = 0;

void speed_change_smooth()
{ // Smooth speed change
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

//----------------------Motor moments------------------------
// The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach(); // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach(); // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);    // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front); // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() // Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void reverse()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw()
{
  // budget_slam();
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void move(float x, float y, float z)
{

  float ang_val_ratios[n];

  inverse_kinematics(x, y, z, ang_val_ratios);

  // int k = 150; // 150 is the maximum speed
  int k = 150;

  left_font_motor.writeMicroseconds(1500 - (ang_val_ratios[0] * k));
  left_rear_motor.writeMicroseconds(1500 - (ang_val_ratios[2] * k));
  right_rear_motor.writeMicroseconds(1500 + (ang_val_ratios[3] * k));
  right_font_motor.writeMicroseconds(1500 + (ang_val_ratios[1] * k));
}

void inverse_kinematics(float vel_x, float vel_y, float omega_z, float *ang_vel_ratio)
{

  float ang_vel[n];
  // fl, fr, bl, br

  float ik_constant = (HALF_LR_WHEEL_TO_WHEEL + FB_WHEEL_TO_WHEEL) * omega_z;

  ang_vel[0] = (vel_x - vel_y - ik_constant) / WHEEL_RADIUS; // fl
  ang_vel[1] = (vel_x + vel_y + ik_constant) / WHEEL_RADIUS; // fr
  ang_vel[2] = (vel_x - vel_y + ik_constant) / WHEEL_RADIUS; // bl
  ang_vel[3] = (vel_x + vel_y - ik_constant) / WHEEL_RADIUS; // br

  float max = 0.0;
  for (int i = 0; i < n; i++)
  {
    float current_abs = fabs(ang_vel[i]);
    if (current_abs > max)
    {
      max = current_abs;
    }
  }
  // BluetoothSerial.print("Max: ");
  // BluetoothSerial.println(max);
  delay(10);
  for (int john = 0; john < n; john++)
  {
    if (max > 0.001)
    {
      ang_vel_ratio[john] = ang_vel[john] / max;
    }
    else
    {
      ang_vel_ratio[john] = 0;
    }
  }
  BluetoothSerial.println("Ratios: " + String(ang_vel_ratio[0], 2) + ", " + String(ang_vel_ratio[1], 2) + ", " + String(ang_vel_ratio[2], 2) + ", " + String(ang_vel_ratio[3], 2));
}

void drive_straight_poc()
{
  int ir_enabled = 0*ir_drive_toggle;
  int gyro_enabled = 1;
  int derivative_enabled = 1;
  // lk its fine without the D term with just PI 120/3

  // float last_print = millis();

  bool wall_proximity = false;

  // PID VALUES
  // float kp_ir = 0.5*ir_enabled;
  float kp_gyro = 120 * gyro_enabled;

  // float ki_ir = 0.001*ir_enabled;
  float ki_gyro = 3 * gyro_enabled;

  float kd_gyro = 5 * derivative_enabled;

  //   // PID VALUES // OLD BEAUTIFUL 150 SPEED_VAL SPEEDS :) SORT OF OK
  float kp_ir = 10*ir_enabled;
  // float kp_gyro = 120 * gyro_enabled;

  float ki_ir = 0*0.001*ir_enabled;
  // float ki_gyro = 3 * gyro_enabled;

  // float kd_gyro = 5 * derivative_enabled;

  float err_ir;
  float err_gyro;

  float ir_u;
  float gyro_u;

  float gyro_read;
  float avg_lr_read;

  // sets the current wall distance to be the r(t)
  // sets current gyro to become the reference angle
  // ReadIRSensors();
  float LR_right = getRightLR();
  float LR_left = getLeftLR();

  if (LR_right > LR_left)
  {
    avg_lr_read = LR_left;
  }
  else
  {
    avg_lr_read = LR_right;
  }

  float lr_initial = avg_lr_read;
  gyro_read = get_rotation_vector_yaw();
  float gyro_initial = gyro_read;

  float speed_val_reborn = 150;


  // loop
  while (!wall_proximity)
  {
    // budget_slam();

    if (getRightSR() < 85 || getLeftSR() < 85)
    {
      wall_proximity = true;
      stop();
      break;
    }
    // if ((getRightSR()+getLeftSR())/2 < 85){
    //   wall_proximity = true;
    //   stop();
    //   break;
    // }

    // if (getUSDistance() < 15) {
    //   wall_proximity = true;
    //   stop();
    //   break;
    // }

    LR_right = getRightLR();
    LR_left = getLeftLR();

    if (LR_right > LR_left)
    {
      avg_lr_read = LR_left;
    }
    else
    {
      avg_lr_read = LR_right;
    }

    gyro_read = get_rotation_vector_yaw();

    // Short range IR sensor outputs Right and Left
    // float sr_right = pow((adcRaw3 / 31299.0), (1.0 / -1.067));
    // float sr_left = pow((adcRaw4 / 1562610.0), (1.0 / -1.98778));

    // error calcs
    err_ir = lr_initial - avg_lr_read;
    err_gyro = angle_diff(gyro_initial, gyro_read);

    // // deadband for stopping the error if its too low // removing for now because idk if its needed
    // if (abs(err_ir) < 1) err_ir = 0;
    // if (abs(err_gyro) < 0.5) err_gyro = 0;

    float d_err = err_gyro - prev_err_gyro;
    prev_err_gyro = err_gyro;

    // integral terms and windup prevention
    integral_sum_ir += err_ir;
    integral_sum_gyro += err_gyro;
    integral_sum_ir = constrain(integral_sum_ir, -1000, 1000);
    integral_sum_gyro = constrain(integral_sum_gyro, -1000, 1000);

    // control effort calcs
    ir_u = kp_ir *  err_ir + ki_ir * integral_sum_ir;
    gyro_u = kp_gyro * err_gyro + ki_gyro * integral_sum_gyro + kd_gyro * d_err;

    // clamping?? idk

    // left_font_motor.writeMicroseconds(1500 - speed_val - ir_u - gyro_u);
    // left_rear_motor.writeMicroseconds(1500 - speed_val + ir_u - gyro_u);
    // right_rear_motor.writeMicroseconds(1500 + speed_val + ir_u - gyro_u);
    // right_font_motor.writeMicroseconds(1500 + speed_val - ir_u - gyro_u);

    left_font_motor.writeMicroseconds(1500 + speed_val - ir_u - gyro_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + ir_u - gyro_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + ir_u - gyro_u);
    right_font_motor.writeMicroseconds(1500 - speed_val - ir_u - gyro_u);

    // DEBUGS
    // if (millis() - last_print > 100) {
    // BluetoothSerial.print("err_gyro: ");
    // BluetoothSerial.println(err_gyro, 4);
    // BluetoothSerial.println();
    //   BluetoothSerial.print("gyro_u: ");
    //   BluetoothSerial.println(gyro_u, 2);
    //   BluetoothSerial.println();
    //   last_print = millis();
    // }

    delay(10); // DELAY ///////////////
  }

  stop();
  // BluetoothSerial.println("YAYAYAYAY drive finished");
  function_complete = true; // FLAG THE COMPLETION OF THIS FUNCTION (for the fake fsm)
}

void drive_tothis_poc_GV(float distance)
{
  // int ir_enabled = 0;
  int gyro_enabled = 1;
  int derivative_enabled = 1;
  float current_US = getUSDistance();
  float target_US_distance = current_US - distance;
  // BluetoothSerial.print("Target distance: ");
  // BluetoothSerial.println(target_US_distance);
  // delay(100);

  // lk its fine without the D term with just PI 120/3

  // float last_print = millis();

  bool wall_proximity = false;

  // PID VALUES
  // float kp_ir = 0.5*ir_enabled;
  float kp_gyro = 120 * gyro_enabled;

  // float ki_ir = 0.001*ir_enabled;
  float ki_gyro = 3 * gyro_enabled;

  float kd_gyro = 5 * derivative_enabled;

  // float err_ir;
  float err_gyro;

  float ir_u;
  float gyro_u;

  float gyro_read;
  // float avg_lr_read;

  // sets the current wall distance to be the r(t)
  // sets current gyro to become the reference angle
  // ReadIRSensors();
  // avg_lr_read = (distLR1 + distLR2) / 2.0; // FIX THIS
  // float lr_initial = avg_lr_read;
  gyro_read = get_rotation_vector_yaw();
  float gyro_initial = gyro_read + inherited_angle;
  // unsigned long fucky = millis();

  unsigned long last_print = millis();

  // loop
  while (!wall_proximity)
  {
    // budget_slam();

    if (fabs(getUSDistance() - target_US_distance) < 1)
    {
      wall_proximity = true;
      stop();
      break;
    }

    // avg_lr_read = (distLR1 + distLR2) / 2.0;
    gyro_read = get_rotation_vector_yaw();

    // Short range IR sensor outputs Right and Left
    // float sr_right = pow((adcRaw3 / 31299.0), (1.0 / -1.067));
    // float sr_left = pow((adcRaw4 / 1562610.0), (1.0 / -1.98778));

    // error calcs
    // err_ir = lr_initial - avg_lr_read;
    err_gyro = angle_diff(gyro_initial, gyro_read);

    // // deadband for stopping the error if its too low // removing for now because idk if its needed
    // if (abs(err_ir) < 1) err_ir = 0;
    // if (abs(err_gyro) < 0.5) err_gyro = 0;

    float d_err = err_gyro - prev_err_gyro;
    prev_err_gyro = err_gyro;

    // integral terms and windup prevention
    // integral_sum_ir += err_ir;
    integral_sum_gyro += err_gyro;
    // integral_sum_ir = constrain(integral_sum_ir, -1000, 1000);
    integral_sum_gyro = constrain(integral_sum_gyro, -1000, 1000);

    // control effort calcs
    // ir_u = kp_ir *  err_ir + ki_ir * integral_sum_ir;
    gyro_u = kp_gyro * err_gyro + ki_gyro * integral_sum_gyro + kd_gyro * d_err;

    // clamping?? idk
    bool forward = true;
    if (getUSDistance() < target_US_distance)
    {
      forward = false;
    }

    if (forward)
    {
      left_font_motor.writeMicroseconds(1500 + speed_val - ir_u - gyro_u);
      left_rear_motor.writeMicroseconds(1500 + speed_val + ir_u - gyro_u);
      right_rear_motor.writeMicroseconds(1500 - speed_val + ir_u - gyro_u);
      right_font_motor.writeMicroseconds(1500 - speed_val - ir_u - gyro_u);
    }
    else
    {
      left_font_motor.writeMicroseconds(1500 - speed_val - ir_u - gyro_u);
      left_rear_motor.writeMicroseconds(1500 - speed_val + ir_u - gyro_u);
      right_rear_motor.writeMicroseconds(1500 + speed_val + ir_u - gyro_u);
      right_font_motor.writeMicroseconds(1500 + speed_val - ir_u - gyro_u);
    }

    // DEBUGS
    if (millis() - last_print > 200)
    {
      BluetoothSerial.print("err_us:");
      BluetoothSerial.println(current_US);
      last_print = millis();
    }

    delay(10); // DELAY ///////////////
  }

  stop();
  // BluetoothSerial.println("YAYAYAYAY drive finished");
  function_complete = true; // FLAG THE COMPLETION OF THIS FUNCTION (for the fake fsm)
}

void drive_tothis_poc(float distance)
{
  // int ir_enabled = 0;
  int gyro_enabled = 1;
  int derivative_enabled = 1;
  float current_US = getUSDistance();
  float target_US_distance = current_US - distance;
  // BluetoothSerial.print("Target distance: ");
  // BluetoothSerial.println(target_US_distance);
  // delay(100);

  // lk its fine without the D term with just PI 120/3

  // float last_print = millis();

  bool wall_proximity = false;

  // PID VALUES
  // float kp_ir = 0.5*ir_enabled;
  float kp_gyro = 120 * gyro_enabled;

  // float ki_ir = 0.001*ir_enabled;
  float ki_gyro = 3 * gyro_enabled;

  float kd_gyro = 5 * derivative_enabled;

  // float err_ir;
  float err_gyro;

  float ir_u;
  float gyro_u;

  float gyro_read;
  // float avg_lr_read;

  // sets the current wall distance to be the r(t)
  // sets current gyro to become the reference angle
  // ReadIRSensors();
  // avg_lr_read = (distLR1 + distLR2) / 2.0; // FIX THIS
  // float lr_initial = avg_lr_read;
  gyro_read = get_rotation_vector_yaw();
  float gyro_initial = gyro_read;
  // unsigned long fucky = millis();

  unsigned long last_print = millis();

  // loop
  while (!wall_proximity)
  {
    // budget_slam();

    if (fabs(getUSDistance() - target_US_distance) < 0.5)
    {
      wall_proximity = true;
      stop();
      break;
    }

    // avg_lr_read = (distLR1 + distLR2) / 2.0;
    gyro_read = get_rotation_vector_yaw();

    // Short range IR sensor outputs Right and Left
    // float sr_right = pow((adcRaw3 / 31299.0), (1.0 / -1.067));
    // float sr_left = pow((adcRaw4 / 1562610.0), (1.0 / -1.98778));

    // error calcs
    // err_ir = lr_initial - avg_lr_read;
    err_gyro = angle_diff(gyro_initial, gyro_read);

    // // deadband for stopping the error if its too low // removing for now because idk if its needed
    // if (abs(err_ir) < 1) err_ir = 0;
    // if (abs(err_gyro) < 0.5) err_gyro = 0;

    float d_err = err_gyro - prev_err_gyro;
    prev_err_gyro = err_gyro;

    // integral terms and windup prevention
    // integral_sum_ir += err_ir;
    integral_sum_gyro += err_gyro;
    // integral_sum_ir = constrain(integral_sum_ir, -1000, 1000);
    integral_sum_gyro = constrain(integral_sum_gyro, -1000, 1000);

    // control effort calcs
    // ir_u = kp_ir *  err_ir + ki_ir * integral_sum_ir;
    gyro_u = kp_gyro * err_gyro + ki_gyro * integral_sum_gyro + kd_gyro * d_err;

    // clamping?? idk
    bool forward = true;
    if (getUSDistance() < target_US_distance)
    {
      forward = false;
    }

    if (forward)
    {
      left_font_motor.writeMicroseconds(1500 + speed_val - ir_u - gyro_u);
      left_rear_motor.writeMicroseconds(1500 + speed_val + ir_u - gyro_u);
      right_rear_motor.writeMicroseconds(1500 - speed_val + ir_u - gyro_u);
      right_font_motor.writeMicroseconds(1500 - speed_val - ir_u - gyro_u);
    }
    else
    {
      left_font_motor.writeMicroseconds(1500 - speed_val - ir_u - gyro_u);
      left_rear_motor.writeMicroseconds(1500 - speed_val + ir_u - gyro_u);
      right_rear_motor.writeMicroseconds(1500 + speed_val + ir_u - gyro_u);
      right_font_motor.writeMicroseconds(1500 + speed_val - ir_u - gyro_u);
    }

    // DEBUGS
    if (millis() - last_print > 200)
    {
      BluetoothSerial.print("err_us:");
      BluetoothSerial.println(current_US);
      last_print = millis();
    }

    delay(10); // DELAY ///////////////
  }

  stop();
  // BluetoothSerial.println("YAYAYAYAY drive finished");
  function_complete = true; // FLAG THE COMPLETION OF THIS FUNCTION (for the fake fsm)
}

void strafe_straight_poc(int direction)
{

  int us_enabled = 1;
  int gyro_enabled = 1;
  int derivative_enabled = 0;
  // lk its fine without the D term with just PI 120/3

  float last_print = millis();

  bool wall_proximity = false;

  // PID VALUES
  // float kp_gyro = 120 * 5 * gyro_enabled;
  // float kp_us = 15 * us_enabled;

  // float ki_gyro = 3 * gyro_enabled;
  // float ki_us = 0.01 * us_enabled;

  // float kd_gyro = 5 * derivative_enabled;

  float kp_gyro = 150 * 5 * gyro_enabled;
  float kp_us = 25 * us_enabled;

  float ki_gyro = 3 * gyro_enabled;
  float ki_us = 0.5 * us_enabled;

  float kd_gyro = 5 * derivative_enabled;

  float err_gyro;
  float err_us;

  float ir_u;
  float gyro_u;
  float us_u;

  float gyro_read;
  float us_read;

  // sets the current wall distance to be the r(t)
  // sets current gyro to become the reference angle
  gyro_read = get_rotation_vector_yaw();
  float gyro_initial = gyro_read;
  us_read = getUSDistance();
  float us_initial = us_read;

  unsigned long exit_timer = millis();

  // loop
  while (!wall_proximity)
  {
    if (direction == 1)
    {
      if (getRightLR() < 150)
      {
        wall_proximity = true;
        stop();
        break;
      }
    }
    else
    {
      if (getLeftLR() < 150)
      {
        wall_proximity = true;
        stop();
        break;
      }
    }

    gyro_read = get_rotation_vector_yaw();

    float sr_right = getRightSR();
    float sr_left = getLeftSR();

    us_read = getUSDistance();

    // exit condition (stop after 800 ms) /// EXIT CONDITION TIMER ::!!::!:!:!:!:!::!:!:!:!::!:!:!:
    if (millis() - exit_timer > 1000)
    {
      wall_proximity = true;
      stop();
      break;
    }

    // error calcs
    err_gyro = angle_diff(gyro_initial, gyro_read);
    err_us = us_initial - us_read;

    // // deadband for stopping the error if its too low // removing for now because idk if its needed
    // if (abs(err_ir) < 1) err_ir = 0;
    // if (abs(err_gyro) < 0.5) err_gyro = 0;

    float d_err = err_gyro - prev_err_gyro;
    prev_err_gyro = err_gyro;

    // integral terms and windup prevention
    integral_sum_gyro += err_gyro;
    integral_sum_us += err_us;
    float int_clamp = 100;
    integral_sum_gyro = constrain(integral_sum_gyro, -int_clamp, int_clamp);
    integral_sum_us = constrain(integral_sum_us, -int_clamp, int_clamp);

    // control effort calcs
    gyro_u = kp_gyro * err_gyro + ki_gyro * integral_sum_gyro + kd_gyro * d_err;
    us_u = kp_us * err_us; // + ki_us * integral_sum_us;

    // // clamping?? idk
    // gyro_u = constrain(gyro_u, -80, 80);

    // get if right or left

    if (direction == 1)
    {
      // strafe right
      left_font_motor.writeMicroseconds(1500 + speed_val - gyro_u - us_u);
      left_rear_motor.writeMicroseconds(1500 - speed_val - gyro_u - us_u);
      right_font_motor.writeMicroseconds(1500 - speed_val - gyro_u + us_u);
      right_rear_motor.writeMicroseconds(1500 + speed_val - gyro_u + us_u);
    }
    else
    {
      // strafe left
      left_font_motor.writeMicroseconds(1500 - speed_val - gyro_u - us_u);
      left_rear_motor.writeMicroseconds(1500 + speed_val - gyro_u - us_u);
      right_font_motor.writeMicroseconds(1500 + speed_val - gyro_u + us_u);
      right_rear_motor.writeMicroseconds(1500 - speed_val - gyro_u + us_u);
    }

    // DEBUGS
    if (millis() - last_print > 100)
    {
      // BluetoothSerial.print("err_gyro: ");
      // BluetoothSerial.println(err_gyro, 4);
      // BluetoothSerial.println();
      // BluetoothSerial.print("us_u: ");
      // BluetoothSerial.println(us_u, 2);
      // BluetoothSerial.println();
      last_print = millis();
    }

    delay(10); // DELAY ///////////////
  }

  stop();
  BluetoothSerial.println("YAYAYAYAY");
  function_complete = true; // FLAG THE COMPLETION OF THIS FUNCTION (for the fake fsm)
}

void strafe_thismuch_poc(int direction, float distance)
{ // 1 is right, 0 is left

  int us_enabled = 1;
  int gyro_enabled = 1;
  int derivative_enabled = 1;

  unsigned long fuck = millis();
  unsigned long last_print = millis();

  bool wall_proximity = false;

  bool sensor_in_range = (getLeftLR() > getRightLR()); // true if right sensor is in range, false if left sensor
                                                       //  PID VALUES
  float kp_gyro = 300 * gyro_enabled;
  float ki_gyro = 1 * gyro_enabled;
  float kd_gyro = 0.001 * derivative_enabled;

  float kp_us = 50 * us_enabled;
  float ki_us = 0.1 * us_enabled;

  float err_gyro, err_us, ir_u, gyro_u, us_u, gyro_read, us_read;

  float target_sensor_distance = 0;
  float start_dist = 0;
  // if (direction && sensor_in_range){
  //   target_sensor_distance = getRightLR() - distance;
  // } else if (direction && !sensor_in_range){
  //   target_sensor_distance = getLeftLR() + distance;
  // } else if (!direction && sensor_in_range){
  //   target_sensor_distance = getRightLR() + distance;
  // } else {
  //   target_sensor_distance = getLeftLR() - distance;
  // }
  if (direction && sensor_in_range)
  {
    start_dist = getRightLR();
    target_sensor_distance = start_dist - distance;
  }
  else if (direction && !sensor_in_range)
  {
    start_dist = getLeftLR();
    target_sensor_distance = start_dist + distance;
  }
  else if (!direction && sensor_in_range)
  {
    start_dist = getRightLR();
    target_sensor_distance = start_dist + distance;
  }
  else
  {
    start_dist = getLeftLR();
    target_sensor_distance = start_dist - distance;
  }

  // sets the current wall distance to be the r(t)
  // sets current gyro to become the reference angle
  gyro_read = get_rotation_vector_yaw();
  us_read = getUSDistance();
  float us_initial = us_read, gyro_initial = gyro_read;
  // delay(100);

  // loop
  while (!wall_proximity && (millis() - fuck < 1000))
  {
    // budget_slam();

    if (sensor_in_range)
    {
      if (fabs(getRightLR() - target_sensor_distance) < 2)
      {
        wall_proximity = true;
        stop();
        break;
      }
    }
    else
    {
      if (fabs(getLeftLR() - target_sensor_distance) < 2)
      {
        wall_proximity = true;
        stop();
        break;
      }
    }

    gyro_read = get_rotation_vector_yaw();
    // float lr_right = getRightLR()*10;
    // float lr_left = getLeftLR()*10;
    us_read = getUSDistance();

    // error calcs
    err_gyro = angle_diff(gyro_initial, gyro_read);
    err_us = us_initial - us_read;

    // // deadband for stopping the error if its too low // removing for now because idk if its needed
    // if (abs(err_ir) < 1) err_ir = 0;
    // if (abs(err_gyro) < 0.5) err_gyro = 0;

    float d_err = err_gyro - prev_err_gyro;
    prev_err_gyro = err_gyro;

    // integral terms and windup prevention
    integral_sum_gyro += err_gyro;
    integral_sum_us += err_us;
    float int_clamp = 100;
    integral_sum_gyro = constrain(integral_sum_gyro, -int_clamp, int_clamp);
    integral_sum_us = constrain(integral_sum_us, -int_clamp, int_clamp);

    // control effort calcs
    gyro_u = kp_gyro * err_gyro + ki_gyro * integral_sum_gyro + kd_gyro * d_err;
    us_u = kp_us * err_us + ki_us * integral_sum_us;

    // // clamping?? idk
    // gyro_u = constrain(gyro_u, -80, 80);

    // get if right or left

    if (direction == 1)
    {
      // strafe right
      left_font_motor.writeMicroseconds(1500 + speed_val - gyro_u - us_u);
      left_rear_motor.writeMicroseconds(1500 - speed_val - gyro_u - us_u);
      right_font_motor.writeMicroseconds(1500 - speed_val - gyro_u + us_u);
      right_rear_motor.writeMicroseconds(1500 + speed_val - gyro_u + us_u);
    }
    else
    {
      // strafe left
      left_font_motor.writeMicroseconds(1500 - speed_val - gyro_u - us_u);
      left_rear_motor.writeMicroseconds(1500 + speed_val - gyro_u - us_u);
      right_font_motor.writeMicroseconds(1500 + speed_val - gyro_u + us_u);
      right_rear_motor.writeMicroseconds(1500 - speed_val - gyro_u + us_u);
    }

    // DEBUGS
    if (millis() - last_print > 200)
    {
      BluetoothSerial.print("t: ");
      BluetoothSerial.println(target_sensor_distance, 3);
      BluetoothSerial.println();

      // delay(10);
      BluetoothSerial.print("c: ");
      if (sensor_in_range)
      {
        BluetoothSerial.println(getRightLR() - target_sensor_distance);
      }
      else
      {
        BluetoothSerial.println(getLeftLR() - target_sensor_distance);
      }
      BluetoothSerial.println();
      last_print = millis();
    }

    delay(10); // DELAY ///////////////
  }

  stop();
  // BluetoothSerial.println("YAYAYAYAY");
  function_complete = true; // FLAG THE COMPLETION OF THIS FUNCTION (for the fake fsm)
}

void turn_n_degrees(int deg)
{
  const float Kp = 180; // calibrate them
  const float Ki = 0.1;
  const float Kd = 0.002;
  const float tolerance = (1.0 * PI) / 180.0; // 2 degrees in radians
  const int max_output = 400;                 // PREVIOUSLY 200 :)
  const int min_power = 60;                   // Adjust this! Find the minimum microsecond offset needed to move the robot.

  const unsigned long required_settle_time = 250;
  unsigned long settle_start_time = 0;
  bool is_settling = false;

  float rad_to_turn = (deg * PI) / 180.0;
  float target_heading = get_rotation_vector_yaw() + rad_to_turn;
  float integral = 0.0;
  float prev_error = 0.0;
  unsigned long last_time = micros();
  unsigned long last_print = millis();
  unsigned long fuck_off = millis();

  while (millis() - fuck_off < 3000)
  {
    // budget_slam();
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0;
    if (dt <= 0.00)
    {
      dt = 0.001;
    }
    last_time = now;

    float current_heading = get_rotation_vector_yaw(); // updates robot_heading
    float error = angle_diff(target_heading, current_heading);
    float abs_error = fabs(error);

    if (abs_error <= tolerance)
    {
      if (!is_settling)
      {
        // Just entered the target zone, start the timer
        settle_start_time = millis();
        is_settling = true;
      }
      else if (millis() - settle_start_time >= required_settle_time)
      {
        // Remained in the target zone long enough, success!
        BluetoothSerial.println("Target reached and settled.");
        break;
      }
    }
    else
    {
      // Fell out of the target zone (overshot), reset the settling timer
      is_settling = false;
    }

    integral += error * dt;
    integral = constrain(integral, -1000.0, 1000.0);

    float derivative = (error - prev_error) / dt;
    prev_error = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if (output > 0 && output < min_power)
    {
      output = min_power;
    }
    else if (output < 0 && output > -min_power)
    {
      output = -min_power;
    }

    float command = constrain(output, -max_output, max_output);

    left_font_motor.writeMicroseconds(1500 - (command));
    left_rear_motor.writeMicroseconds(1500 - (command));
    right_rear_motor.writeMicroseconds(1500 - (command));
    right_font_motor.writeMicroseconds(1500 - (command));

    // if (millis() - last_print > 200) {
    //       // BluetoothSerial.print("output: ");
    //       //   BluetoothSerial.println(output);
    //   BluetoothSerial.print("Turn err: ");
    // BluetoothSerial.println(error * 180.0 / PI, 2);
    // last_print = millis();
    //     }
    delay(10);
  }

  stop();
}
