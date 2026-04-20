// #include <cmath>

#include "homing.h"


void G28()
{
  static unsigned long previous_millis;
  previous_millis = millis();
  float error = 100;
  
  while(error > 5){
    if ((millis() - previous_millis) > 100){
      previous_millis = millis();
      float LSR = getLeftSR();
      float RSR = getRightSR();
      //print sensor valaues for debugging
      // BluetoothSerial.print("LSR: "); 
      // BluetoothSerial.print(LSR);
      // BluetoothSerial.print(" RSR: ");
      // BluetoothSerial.println(RSR);
      if(LSR > 10 && RSR > 10){
        BluetoothSerial.println("too far lol");
      } else {
        float measurements[3] = {0};
        Align(measurements);
        BluetoothSerial.print("x: ");
        BluetoothSerial.print(measurements[0]);
        BluetoothSerial.print(" y: ");
        BluetoothSerial.print(measurements[1]);
        BluetoothSerial.print(" angle: ");
        BluetoothSerial.println(measurements[2]);
        break;
      }
    }
  }
}

void Align(float *array)
{
  float LSR = getLeftSR();
  float RSR = getRightSR();

  float angle = PI/2 - atan((SR_SPACING) / (RSR - LSR));
  if (angle > PI/2){
    angle = angle -PI;
  }
  float distance = ((LSR+RSR)/2) * cos(angle) - 5;
  float x = distance * cos(angle);
  float y = distance * sin(angle);
  float rot_angle = angle;

  array[0] = x;
  array[1] = y;
  array[2] = rot_angle;

}

// void drive_straight_poc(){
//   bool wall_proximity = false;

//   float speed_val_reborne = 150;

//   float kp_x =      2     *speed_val_reborne;
//   float kp_y =      2     *speed_val_reborne;
//   float kp_gyro =   2     *speed_val_reborne;

//   // float ki_x = 0.01;
//   // float ki_y = 0.01;
//   // float ki_gyro = 0.01;

//   float err_x;
//   float err_y;
//   float err_gyro;

//   // float integral_sum_x;
//   // float integral_sum_y;
//   // float integral_sum_gyro;

//   float x_u;
//   float y_u;
//   float gyro_u;

//   // loop
//   while (!wall_proximity){
//   //BluetoothSerial.println("i print every time drive loops");


//     float movement[3] = {0};
//     Align(movement);
//     err_x = movement[0];
//     err_y = movement[1];  
//     err_gyro = movement[2];
//     // control effort calcs
//     x_u = kp_x * err_x;
//     y_u = kp_y *  err_y;
//     gyro_u = kp_gyro * err_gyro;
//     // print control efforts for debugging
//     BluetoothSerial.print("x_u: ");
//     BluetoothSerial.print(x_u);
//     BluetoothSerial.print(" y_u: ");
//     BluetoothSerial.print(y_u);
//     BluetoothSerial.print(" gyro_u: ");
//     BluetoothSerial.println(gyro_u);

//     // if (x_u + y_u + gyro_u < 1) {
//     //   wall_proximity = true;
//     //   stop();
//     //   BluetoothSerial.println("stopped in stop condition");
//     //   delay(2000);
//     //   break;
//     // }

//     // clamping?? idk

//     left_font_motor.writeMicroseconds(1500 - x_u - y_u + gyro_u);
//     left_rear_motor.writeMicroseconds(1500 - x_u + y_u + gyro_u);
//     right_rear_motor.writeMicroseconds(1500 + x_u + y_u + gyro_u);
//     right_font_motor.writeMicroseconds(1500 + x_u - y_u + gyro_u);

//     //add reasonable delay to prevent control fuckups
//     delay(100);
//   }
        
//   stop();
// }
// void G28()
// {
//   float distReadings[TURNING_SAMPLES] = {0};
//   // float distLeftIRReadings[TURNING_SAMPLES] = {0};
//   // float distRightIRReadings[TURNING_SAMPLES] = {0};
//   float theta[TURNING_SAMPLES] = {0};

//   static unsigned long previous_millis;

//   int i = 0;

  

//   robot_heading = 0;
//   ccw(); // starting spinning outsisde loop

//   while (i < TURNING_SAMPLES)
//   {
//     GYRO_reading(); // more accurate to read this more often
//     if (millis() - previous_millis > 100)
//     { // Arduino style 500ms timed execution statement
//       previous_millis = millis();

//       // distLeftIRReadings[i] = getLeftSR();
//       // distRightIRReadings[i] = getRightSR();
//       distReadings[i] = getUSDistance();
//       theta[i] = robot_heading;

//       i++;
//     }
//   }
//   stop();

//   // budget_SLAM(distReadings, distLeftIRReadings, distRightIRReadings, theta, turning_speed_index);

//   int minIndex = FindMinIndex(distReadings, TURNING_SAMPLES);
//   float desiredTheta = theta[minIndex];

//   // shortest path turn
//   while (true)
//   {
//     GYRO_reading();
//     float error = desiredTheta - robot_heading;

//     while (error > PI)
//       error -= (2 * PI);
//     while (error < -PI)
//       error += (2 * PI);

//     if (abs(error) < 0.1)
//       break; // close enough

//     if (error > 0)
//     {
//       ccw();
//     }
//     else
//     {
//       cw();
//     }
//   }
//   stop();

//   delay(200); // debugging staging ---------------------------------------------------------------------------------

//   BluetoothSerial.println("for till ir");
//   forward();
//   while (true)
//   {
//     if (millis() - previous_millis > 200)
//     {
//       previous_millis = millis();
//       if (getRightLR() < 29 || getLeftSR() < 29)
//       {
//         break;
//       }
//     }
//   }

//   // stop();

//   // use ir sensors for wiggle (not ultrasonic)
//   BluetoothSerial.println("wiggle");
//   unsigned long alignTimer = millis();
//   speed_val = 50;
//   while (millis() - alignTimer < 1000) // try to align for 1 second
//   {
//     float distSR1 = getRightSR();
//     float distSR2 = getLeftSR();
//     if (abs(distSR1 - distSR2) < 1)
//     {
//       break;
//     }
//     if (distSR1 > distSR2)
//     {
//       cw();
//     }
//     else
//     {
//       ccw();
//     }
//   }
//   stop();
//   speed_val = 150;
//   delay(100);
//   BluetoothSerial.println("forward");
//   forward();
//   while (true)
//   {
//     if (millis() - previous_millis > 100)
//     {
//       previous_millis = millis();

//       float distSR1 = getRightSR();
//       float distSR2 = getLeftSR();
//       if (distSR1 < 10 || distSR2 < 10)
//       {
//         break;
//       }
//     }
//   }
//   stop();
//   int desiredThetaInx = getIndex(theta, TURNING_SAMPLES, desiredTheta);
//   float left_wall_dist = distReadings[desiredThetaInx - (TURNING_SAMPLES / 4)];
//   float right_wall_dist = distReadings[desiredThetaInx + (TURNING_SAMPLES / 4)];
//   BluetoothSerial.println("align");
//   AlignWithWall(2);

//   int ang_error; // not being used? --------------------------------------

//   BluetoothSerial.println("turn"); // this needs to be fixed so that it correctly aligns iut self pointing the towards the long edge
//   while (true)
//   {
//     GYRO_reading();
//     if (left_wall_dist > right_wall_dist)
//     {
//       quarter_turn(2);
//     }
//     else
//     {
//       quarter_turn(1);
//     }
//   }

//   stop(); // sets up x, y and heading values !!!!!!!!
//   robotX = 0;
//   robotY = 0;
//   robot_heading = 0;
// }
