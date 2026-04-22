#include "farming.h"
#include "sensors.h"
#include "helpers.h"
#include "motors.h"

// void farming_init()
// {

//   // turret_motor.write(90); // point the servo left
//   int num_tramlines = int(TABLE_WIDTH) % JOHN_ROBOT_WIDTH; // find out how mant tramlines there are
//   float laneTargets[num_tramlines];

//   for (int john = 1; john <= num_tramlines; john++)
//   {
//     laneTargets[john - 1] = (TABLE_WIDTH / num_tramlines) * john;
//   }

//   // = {20, 44, 68, 92, 116}; // this needs to be variable, not hardcoded in to the code, like it reads with the ultrasonic sensor how far away it is and then divides that up into tramlines etc

//   // --- PID --- PD implementation because error will always exist (Ki inappropriate)
//   Kp = 2.0;
//   Ki = 0.001;
//   Kd = 0.01;

//   ortho_error = 0;
//   prev_error = 0;

//   baseSpeed = 120;
//   maxCorrection = 80;
// }

void farming() {
    BluetoothSerial.println("Starting farming trace...");

    // 1. Determine which direction the rest of the course is.
    // 1 = Right, 0 = Left (matching your strafe functions)
    int strafe_dir = 1; 
    if (getLeftLR() > getRightLR()) {
        strafe_dir = 0; // Left side is open
    }

    // 2. Define Thresholds
    const float LANE_WIDTH = 250.0;           // mm to strafe for each lane
    const float REAR_WALL_TARGET = 1750.0;    // mm target for US sensor when driving backwards (course is 1991mm long)
    const float SIDE_WALL_THRESHOLD = 250.0;  // mm distance to far wall to consider course complete

    bool course_completed = false;
    bool driving_forward = true;

    while (!course_completed) {
        
        // --- 1. TRACE THE LANE ---
        if (driving_forward) {
            BluetoothSerial.println("Farming: Driving Forward...");
            drive_straight_poc(); // Drives until SR sensors < 100mm
        } else {
            BluetoothSerial.println("Farming: Driving Backwards...");
            drive_tothis_poc(REAR_WALL_TARGET); // Drives backward until US sensor reads ~1750mm
        }

        delay(300); // Allow momentum to settle 

        // --- 2. CHECK IF WE REACHED THE END OF THE COURSE ---
        // Look at the LR sensor in the direction we are strafing.
        float side_dist = (strafe_dir == 1) ? getRightLR() : getLeftLR();
        if (side_dist < SIDE_WALL_THRESHOLD) {
            BluetoothSerial.println("Course fully traced. Farming complete.");
            course_completed = true;
            break; 
        }

        // --- 3. STRAFE TO NEXT LANE ---
        BluetoothSerial.println("Farming: Strafing to next lane...");
        strafe_thismuch_poc(strafe_dir, LANE_WIDTH);
        
        delay(300); // Allow momentum to settle before switching axis

        // Flip direction for the next trace
        driving_forward = !driving_forward;
    }
    
    stop();
}