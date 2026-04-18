
#include "config.h"
#include "sensors.h"
#include "motors.h"
// Code Starts ----------------------------------------------------------------------------------------------------
int n = 4;
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

void quarter_turn(int cw_ccw_mode)
{
  unsigned long fuckthis = millis();
  unsigned long lastPrint = 0; // Timer for serial output
  float accumulated_angle;
  while (millis() - fuckthis < 5000)
  {
    GYRO_reading();
    accumulated_angle += rad;

    if (cw_ccw_mode == 1)
      cw();
    else
      ccw();

    // ONLY print every 100ms to avoid crashing the serial buffer
    if (millis() - lastPrint > 100)
    {
      BluetoothSerial.print("Heading: ");
      BluetoothSerial.println(accumulated_angle);
      lastPrint = millis();
    }
    if (abs(accumulated_angle) >= 1.57)
    {
      break;
    }
  }
  stop();
}

void move(float x, float y, float z)
{

  float ang_val_ratios[n];

  inverse_kinematics(x, y, z, ang_val_ratios);

  // int k = 150; // 150 is the maximum speed
  int k = 400;

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
  ang_vel[2] = (vel_x + vel_y - ik_constant) / WHEEL_RADIUS; // bl
  ang_vel[3] = (vel_x - vel_y + ik_constant) / WHEEL_RADIUS; // br
  BluetoothSerial.print(ang_vel[0]);
  BluetoothSerial.print(ang_vel[1]);
  BluetoothSerial.print(ang_vel[2]);
  BluetoothSerial.print(ang_vel[3]);

  float max = 0.0;
  for (int john = 0; john < n; john++)
  {
    if (abs(ang_vel[john]) > max)
    {
      max = abs(ang_vel[john]);
    }
  }
  BluetoothSerial.println("Max: " + String(max));
  delay(1000);
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
