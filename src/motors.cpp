

#include "motors.h"
#include "helpers.h"
#include "sensors.h"

// Code Starts ----------------------------------------------------------------------------------------------------
int n = 4;
float integral_sum_ir;
float integral_sum_gyro;
float integral_sum_us;
bool function_complete = false;

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

  float integral_sum_ir=0;
  float integral_sum_gyro =0;
  float ang_val_ratios[n];

  int ir_enabled = 1;
  int gyro_enabled = 1;
  int derivative_enabled = 1;
  // lk its fine without the D term with just PI 120/3

  float last_print = millis();

  bool wall_proximity = false;

  // PID VALUES
  float kp_ir = 0.5 * ir_enabled;
  float kp_gyro = 120 * gyro_enabled;

  float ki_ir = 0.001 * ir_enabled;
  float ki_gyro = 3 * gyro_enabled;

  float kd_gyro = 5 * derivative_enabled;

  float ir_u, gyro_u, gyro_read, avg_lr_read,err_ir, err_gyro;

  // sets the current wall distance to be the r(t)
  // sets current gyro to become the reference angle
  avg_lr_read = (getLeftLR() + getRightLR()) / 2.0;
  float lr_initial = avg_lr_read;
  gyro_read = get_rotation_vector_yaw();
  float gyro_initial = gyro_read;

  // timing for derivative
  // unsigned long last_time = micros();

  // loop
  while (!wall_proximity)
  {

    // unsigned long now = micros();
    // float dt = (now - last_time) / 1000000.0; // convert to seconds
    // last_time = now;

    avg_lr_read = (getLeftLR() + getRightLR()) / 2.0;
    gyro_read = get_rotation_vector_yaw();

    // Short range IR sensor outputs Right and Left
    // float sr_right = pow((adcRaw3 / 31299.0), (1.0 / -1.067));
    // float sr_left = pow((adcRaw4 / 1562610.0), (1.0 / -1.98778));
    float sr_right = getRightSR();
    float sr_left = getLeftSR();

    // STOP CONDITION
    if (sr_right < 60 || sr_left < 60)
    {
      wall_proximity = true;
      stop();
      delay(2000); // why 2s delay?
      break;
    }

    // error calcs
    err_ir = lr_initial - avg_lr_read;
    err_gyro = angle_diff(gyro_initial, gyro_read);

    // // deadband for stopping the error if its too low // removing for now because idk if its needed
    // if (abs(err_ir) < 1) err_ir = 0;
    // if (abs(err_gyro) < 0.5) err_gyro = 0;

    // float d_err = 0;
    // if (dt>0){
    //   d_err = (err_gyro - prev_err_gyro) / dt;
    // }
    float d_err = err_gyro - prev_err_gyro;
    prev_err_gyro = err_gyro;

    // integral terms and windup prevention
    // integral_sum_ir += (err_ir*dt);
    // integral_sum_gyro += (err_gyro*dt);
    integral_sum_ir += err_ir;
    integral_sum_gyro += err_gyro;
    integral_sum_ir = constrain(integral_sum_ir, -1000, 1000);
    integral_sum_gyro = constrain(integral_sum_gyro, -1000, 1000);

    // control effort calcs
    ir_u = (kp_ir * err_ir) + (ki_ir * integral_sum_ir);
    gyro_u = (kp_gyro * err_gyro) + (ki_gyro * integral_sum_gyro) + (kd_gyro * d_err);

    // clamping?? idk
    // inverse_kinematics(1, 0, 0, ang_val_ratios); // this is just to get the ratios for strafing and forward movement, the actual speed is determined by the control efforts below
    // int k = 150;


    // left_font_motor.writeMicroseconds(1500 - (ang_val_ratios[0] * k) - ir_u - gyro_u);
    // left_rear_motor.writeMicroseconds(1500 - (ang_val_ratios[2] * k) + ir_u - gyro_u);
    // right_rear_motor.writeMicroseconds(1500 + (ang_val_ratios[3] * k) + ir_u - gyro_u);
    // right_font_motor.writeMicroseconds(1500 + (ang_val_ratios[1] * k) - ir_u - gyro_u);

    left_font_motor.writeMicroseconds(1500 - speed_val - ir_u - gyro_u);
    left_rear_motor.writeMicroseconds(1500 - speed_val + ir_u - gyro_u);
    right_rear_motor.writeMicroseconds(1500 + speed_val + ir_u - gyro_u);
    right_font_motor.writeMicroseconds(1500 + speed_val - ir_u - gyro_u);

    // DEBUGS
    if (millis() - last_print > 100)
    {
      // BluetoothSerial.print("err_gyro: ");
      // BluetoothSerial.println(err_gyro, 4);
      // BluetoothSerial.println();
      // BluetoothSerial.print("gyro_u: ");
      // BluetoothSerial.println(gyro_u, 2);
      // BluetoothSerial.println();
      last_print = millis();
    }

    // delay(10); // DELAY ///////////////
  }

  stop();
  BluetoothSerial.println("YAYAYAYAY");
}

void strafe_straight_poc(){

  int us_enabled = 1;
  int gyro_enabled = 1;
  int derivative_enabled = 0;
  // lk its fine without the D term with just PI 120/3

  float last_print = millis();

  bool wall_proximity = false;

  // PID VALUES
  float kp_gyro = 120*5*gyro_enabled;
  float kp_us = 15*us_enabled;

  float ki_gyro = 3*gyro_enabled;
  float ki_us = 0.01*us_enabled;

  float kd_gyro = 5*derivative_enabled;

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

  // loop
  while (!wall_proximity){

    gyro_read = get_rotation_vector_yaw();

    float sr_right = getRightSR();
    float sr_left = getLeftSR();

    us_read = getUSDistance();

    // STOP CONDITION
    if (sr_left < 60 || sr_right < 60) {
      wall_proximity = true;
      stop();
      delay(2000);
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

    left_font_motor.writeMicroseconds(  1500 + speed_val - gyro_u  - us_u);
    left_rear_motor.writeMicroseconds(  1500 - speed_val - gyro_u  - us_u);
    right_font_motor.writeMicroseconds( 1500 - speed_val - gyro_u  + us_u);
    right_rear_motor.writeMicroseconds( 1500 + speed_val - gyro_u  + us_u);

    // DEBUGS 
    if (millis() - last_print > 100) {
      // BluetoothSerial.print("err_gyro: ");
      // BluetoothSerial.println(err_gyro, 4);
      // BluetoothSerial.println();
      BluetoothSerial.print("us_u: ");
      BluetoothSerial.println(us_u, 2);
      BluetoothSerial.println();
      last_print = millis();
    }


    delay(10); // DELAY ///////////////
  }
        
  stop();
  BluetoothSerial.println("YAYAYAYAY");
}

void turn_n_degrees(int deg)
{
  const float Kp = 110.0;
  const float Ki = 1;
  const float Kd = 0.01;
  const float tolerance = (2.0 * PI) / 180.0; // 2 degrees in radians
  const int max_output = 150;
  float last_print = millis();

  const unsigned long required_settle_time = 250;
  unsigned long settle_start_time = 0;
  bool is_settling = false;

  float rad_to_turn = (deg * PI) / 180.0;
  float target_heading = get_rotation_vector_yaw() + rad_to_turn;
  float integral = 0.0;
  float prev_error = 0.0;
  unsigned long start_time = millis(); 
  unsigned long last_time = micros();

  while (true)
  {
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0;
    if (dt <= 0.0)
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
  
    float command = constrain(output, -max_output, max_output);

    left_font_motor.writeMicroseconds(1500 - ( command));
    left_rear_motor.writeMicroseconds(1500 - ( command));
    right_rear_motor.writeMicroseconds(1500 - (command));
    right_font_motor.writeMicroseconds(1500 - ( command));

        if (millis() - last_print > 200) {
          BluetoothSerial.print("output: ");
            BluetoothSerial.println(output);
    BluetoothSerial.print("Turn err: ");
    BluetoothSerial.println(error * 180.0 / PI, 2);
        }
   
  }

  stop();
}

// void quarter_turn(int cw_ccw_mode)
// {
//   unsigned long fuckthis = millis();
//   unsigned long lastPrint = 0; // Timer for serial output
//   float accumulated_angle;
//   while (millis() - fuckthis < 5000)
//   {
//     GYRO_reading();
//     accumulated_angle += rad;

//     if (cw_ccw_mode == 1)
//       cw();
//     else
//       ccw();

//     // ONLY print every 100ms to avoid crashing the serial buffer
//     if (millis() - lastPrint > 100)
//     {
//       BluetoothSerial.print("Heading: ");
//       BluetoothSerial.println(accumulated_angle);
//       lastPrint = millis();
//     }
//     if (abs(accumulated_angle) >= 1.57)
//     {
//       break;
//     }
//   }
//   stop();
// }