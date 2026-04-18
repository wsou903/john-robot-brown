#include "farming.h"

void farming_init()
{

  turret_motor.write(90); // point the servo left
  float field_width = getUSDistance();

  int num_tramlines = int(field_width) % JOHN_ROBOT_WIDTH; // find out how mant tramlines there are
  float laneTargets[num_tramlines];

  for (int john = 1; john <= num_tramlines; john++)
  {
    laneTargets[john - 1] = (field_width / num_tramlines) * john;
  }

  // = {20, 44, 68, 92, 116}; // this needs to be variable, not hardcoded in to the code, like it reads with the ultrasonic sensor how far away it is and then divides that up into tramlines etc

  // --- PID --- PD implementation because error will always exist (Ki inappropriate)
  Kp = 2.0;
  Ki = 0.001;
  Kd = 0.01;

  ortho_error = 0;
  prev_error = 0;

  baseSpeed = 120;
  maxCorrection = 80;
}

void Farming()
{

  // This code traces 5 straight lines (lanes) across the course
  // it assumes we start completely straight, and each time Farming() is called
  // we increment the lane that we are tracing for 5 evenly spaced lines
  // current issues is that it has a reference list for wall distances
  // from the left wall, 20, 44, 68, etc... to keep constant but because the
  // IR sensors only measure 80cm max, we probably need to swap sensors after
  // lane 3. I dont do this atm so the code sort of breaks

  float initial_dr = 0;
  float initial_dl = 0;
  float tramline_distance = 0;
  enum
  {
    LEFT,
    RIGHT
  } referenceWall; // DEFAULTS LEFT

  currentLane = 0; // MAKE GLOBAL

  initial_dr = getRightLR(); // RIGHT
  initial_dl = getLeftLR();  // LEFT

  if (initial_dl < initial_dr)
  {
    referenceWall = LEFT;
  }
  else
  {
    referenceWall = RIGHT;
  }

  tramline_distance = getUSDistance(); // initial forward

  bool end_of_tramline = false; // bool for if we've hit a proximity to the target wall

  while (end_of_tramline == false)
  {

    if (getUSDistance() < 10)
    { // STOP CONDITION
      stop();
      end_of_tramline = true;
      currentLane++; // INCREMENTS THE LANE WE ARE IN FOR THE NEXT RUN
      break;
    }

    float targetDist = laneTargets[currentLane];

    if (referenceWall == LEFT)
    {
      ortho_error = targetDist - getRightLR();
    }
    else
    {
      // ortho_error = distLR1 - targetDist;
      ortho_error = targetDist - getLeftLR();
      // FIX LATER this is to avoid a bug where laneTargets asks for say, 92cm, from left wall, and the closest sensor is right with 38cm, which we should accept as the new reference, but we dont at the moment, so it will then turn left and overshoot to the wrong wall to make the right sensor read 92cm, which is not what we want
    }

    // ########## BELOW IS FULLY CHATTED
    // --- PD CONTROL ---
    float derivative = ortho_error - prev_error;
    float correction = Kp * ortho_error + Kd * derivative;

    // clamp correction
    if (correction > maxCorrection)
      correction = maxCorrection;
    if (correction < -maxCorrection)
      correction = -maxCorrection;

    prev_error = ortho_error;

    // --- MOTOR SPEED MIXING ---
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // --- APPLY TO MOTORS (preserving your convention) ---
    left_font_motor.writeMicroseconds(1500 - leftSpeed);
    left_rear_motor.writeMicroseconds(1500 - leftSpeed);
    right_rear_motor.writeMicroseconds(1500 + rightSpeed);
    right_font_motor.writeMicroseconds(1500 + rightSpeed);

    delay(20); // control loop timing (~50Hz)

  } // END OF WHILE LOOP
}