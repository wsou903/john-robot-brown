#include "helpers.h"
#include "motors.h"
#include "sensors.h"

// THIS IS TO STOP WRAPAROUND ON THE GYRO READ
float angle_diff(float target, float current)
{
  float diff = target - current;
  while (diff > PI)
    diff -= 2 * PI;
  while (diff < -PI)
    diff += 2 * PI;
  return diff;
}

int FindMinIndex(float arr[], int n)
{
  float min = arr[0];
  int mini = 0; // Min Index
  for (int i = 1; i < n; i++)
  {

    // Update m if arr[i] is smaller
    if (arr[i] < min)
    {
      min = arr[i];
      mini = i;
    }
  }
  return mini;
}

int getIndex(float arr[], int n, float desired)
{
  float minDiff = 999.0;
  int bestIndex = 0;
  for (int i = 0; i < n; i++)
  {
    float diff = abs(arr[i] - desired);
    if (diff < minDiff)
    {
      minDiff = diff;
      bestIndex = i;
    }
  }
  return bestIndex;
}

void fast_flash_double_LED_builtin()
{ // Fast LED flash program
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis)
  {
    indexer++;
    if (indexer > 4)
    {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    }
    else
    {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{ // slow LED flash program
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000)
  {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void AlignWithWall(int lr_sr_mode)
{

  float Kp = 2.0;
  float Ki = 0.01;
  float Kd = 0.001;
  float tolerance = 1.0; // cm difference between sensors allowed
  // int maxTurn = 60;
  // int minTurn = 25;        // overcome motor deadzone if needed
  float cumulative_error = 0;
  float prev_error = 0;
  float angle_error = 0;

  unsigned long giveUpTimer = millis(); // this makes it give up if it cant figure it out
  unsigned long lastTime = giveUpTimer;
  while (millis() - giveUpTimer < 5000)
  {

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // in seconds
    lastTime = now;

    if (lr_sr_mode == 1)
    {
      angle_error = getRightLR() - getLeftLR();
    }
    else
    {
      angle_error = getRightSR() - getLeftSR();
    }

    // stop condition
    if (abs(angle_error) < tolerance)
    {
      stop();
      // ############################################ WE CAN ZERO THE IMU HERE #####################################
      robot_heading = 0.0; // it is zerod --------------------------------------------------------------------------------------------------------------------
      break;
    }

    cumulative_error += (angle_error * dt);
    float d_error = (angle_error - prev_error) / dt;
    float turn = (Kp * angle_error) + (Ki * cumulative_error) + (Kd * d_error);
    prev_error = angle_error;

    // clamp
    // if (turn > maxTurn) turn = maxTurn;
    // if (turn < -maxTurn) turn = -maxTurn;

    // ensure minimum torque to actually move
    // if (turn > 0 && turn < minTurn) turn = minTurn;
    // if (turn < 0 && turn > -minTurn) turn = -minTurn;

    // rotate in place
    int leftSpeed = -turn;
    int rightSpeed = turn;

    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    left_font_motor.writeMicroseconds(1500 - leftSpeed);
    left_rear_motor.writeMicroseconds(1500 - leftSpeed);
    right_rear_motor.writeMicroseconds(1500 + rightSpeed);
    right_font_motor.writeMicroseconds(1500 + rightSpeed);

    delay(20);
  }
  stop();
}