#include "homing.h"


float robot_heading_global = 0.0;

void G28()
{
  // BluetoothSerial.println("driving to wall");
  sweep(); // sweep to find the wall and turn towards it
  drive_straight_poc();
  delay(20);
  AlignWithWall();
  delay(20);
  // BluetoothSerial.println("Aligned!!!");
  bool direction = 1; //true is turning right, false is turning left
  if(getLeftLR() < getRightLR()){
    direction = 0;
  }
  if(direction){
    turn_n_degrees(-90);
  } else {
    turn_n_degrees(90);
  }
  drive_straight_poc();
  AlignWithWall();
  if (direction){
    turn_n_degrees(-90);
    delay(20);
    if (getUSDistance() < 150){
      turn_n_degrees(-90);
    }
  } else {
    turn_n_degrees(90);
    delay(20);
    if (getUSDistance() < 150){
      turn_n_degrees(90);
    }
  }
  robotX=0;
  robotY=0;
  robot_heading=0;
  robot_heading_global = get_rotation_vector_yaw(); // set the global heading to the current yaw after homing, so that all future readings are relative to this starting orientation
  // strafe_straight_poc(direction);
  // AlignWithWall();
  // if (getLeftLR() > 750){
  //   turn_n_degrees(90);
  // } else {
  //   turn_n_degrees(180);
  // }
}

void sweep() {
    // 1. Make these 'const' so the array size is known at compile time
    const int spin_time = 2500;   // ms
    const int sampling_rate = 50; // ms
    const int n = spin_time / sampling_rate; // 200 elements

    int i = 0;
    
    // Arrays can now be legally initialized with {0} because 'n' is a constant
    float distances[n] = {0};
    float angles[n] = {0};
    
    // 2. Remove 'static' and use 'unsigned long' (millis() returns unsigned long)
    unsigned long timing = millis();

    ccw(); // Start spinning
    
    while (i < n) {
        if (millis() - timing >= sampling_rate) {
            timing = millis();
            distances[i] = getUSDistance();
            angles[i] = get_rotation_vector_yaw();
            i++;
        }
    }
    stop(); // Stop spinning
    delay(20); // Small delay to ensure the robot has stopped before processing data

    // 3. Find min distance index
    int turn_index = find_min_index(distances, n);

    // 4. Calculate relative turn angle (Target Angle - Current Angle)
    float target_angle = angles[turn_index];
    float current_angle = get_rotation_vector_yaw();
    float turn_amount = target_angle - current_angle; 

    turn_n_degrees(turn_amount * 180.0 / PI); // Convert radians to degrees for the turn function
}

int find_min_index(float *array, int size){
  int min_index = 0;
  for (int i = 1; i < size; i++){
    if (array[i] < array[min_index]){
      min_index = i;
    }
  }
  return min_index;
}

void AlignWithWall()
{
  float movement[2] = {0};
  bool angle_happy = false;
  bool distance_happy = false;
  bool aligned = false;
  while (!aligned)
  {
    delay(20); // add a small delay to prevent overwhelming the sensors and control system
    Align_calc(movement);
    if (fabs(movement[0] - 65) < 7.5)
    {
      // BluetoothSerial.print("distance happy: ");
      // BluetoothSerial.println(movement[0]);
      distance_happy = true;
    }
    else
    {
      // BluetoothSerial.print("distance sad, moving: ");
      // BluetoothSerial.println(movement[0]);
      speed_val = 75;
      drive_tothis_poc((movement[0]/10.0) - 6.5); // need a version of this that moves a set distance, will have to make this for the farming function anyways, alternatively, just make the stop condition for this work in all cases
      speed_val = 150;
      distance_happy = false; // if we had to move, we might need to move again after checking angle, so reset this flag
      continue; // skip the angle check this loop, we want to check angle after we have
    }
    // delay(20);
    if (fabs(movement[1]) < 0.05) // ~3 degrees, adjust as necessary
    {
      // BluetoothSerial.print("angle happy: ");
      // BluetoothSerial.println(movement[1]);
      angle_happy = true;
    }
    else 
    {
      // BluetoothSerial.print("angle sad, rotating: ");
      // BluetoothSerial.println(movement[1]);
      turn_n_degrees(movement[1] * 180.0 / PI);
      angle_happy = false; // if we had to turn, we might need to turn again after checking distance, so reset this flag
      continue; // skip the distance check this loop, we want to check distance after we have the correct angle
    }
    // delay(50);
    if (angle_happy && distance_happy)
    {
      aligned = true;
      // BluetoothSerial.println("aligned with wall");
    }
  }
  

}

void Align_calc(float *array)
{
  const unsigned long delay_millis = 1000;
  const unsigned long polling_rate = 10;
  const unsigned long loops = delay_millis / polling_rate;

  float leftFrontSum = 0.0f;
  float rightFrontSum = 0.0f;

  unsigned long next_sample_time = millis();
  unsigned long sample_count = 0;

  // 1. Collect and add up the readings immediately
  while (sample_count < loops) 
  {
    if (millis() - next_sample_time >= polling_rate)
    {
      next_sample_time = millis();
      
      // Add the new reading directly to the running sum
      leftFrontSum += getLeftSR();
      rightFrontSum += getRightSR();
      
      ++sample_count;
    }
  }

  float leftFrontAverage = 0.0f;
  float rightFrontAverage = 0.0f;

  // 2. Divide the total sum by the number of loops to get the average
  if (loops > 0)
  {
    leftFrontAverage = leftFrontSum / loops;
    rightFrontAverage = rightFrontSum / loops;
  }

 // Assuming left = positive angle. Swap the left/right variables in atan2f if you need right = positive.
const float turn_angle = atan2f(leftFrontAverage - rightFrontAverage, SR_SPACING);

// Subtract the physical robot offset first, THEN project to the perpendicular distance
array[0] = (((leftFrontAverage + rightFrontAverage) * 0.5f) - 5.0f) * cosf(turn_angle);
array[1] = turn_angle;
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
