#include "farming.h"
#include "sensors.h"
#include "helpers.h"
#include "motors.h"
#include "homing.h"

void farming()
{
    // BluetoothSerial.println("Starting farming trace...");

    // 1. Determine which direction the rest of the course is.
    // 1 = Right, 0 = Left (matching your strafe functions)
    int strafe_dir = 1;
    if (getLeftLR() > getRightLR())
    {
        strafe_dir = 0; // Left side is open
    }

    // 2. Define Thresholds
    const float LANE_WIDTH = 100.0;     // mm to strafe for each lane
    const float REAR_WALL_TARGET = 155; // mm target for US sensor when driving backwards (course is 1991mm long)
    const float FWD_WALL_TARGET = 10;
    // const float STARTING_US_DIST = getUSDistance();
    // delay(100);
    BluetoothSerial.print("rear wall target: ");
    BluetoothSerial.println(REAR_WALL_TARGET);
    // delay(100);
    const float SIDE_WALL_THRESHOLD = 100; // mm distance to far wall to consider course complete

    bool course_completed = false;
    bool driving_forward = true;
    int forward_counter = 0;

    while (!course_completed)
    {

        // --- 1. TRACE THE LANE ---
        if (driving_forward)
        {
            // BluetoothSerial.println("Farming: Driving Forward...");
            // drive_straight_poc(); // Drives until SR sensors < 100mm
            drive_tothis_poc(FWD_WALL_TARGET);
            // AlignWithWall();
            forward_counter++;
            if (forward_counter % 2 == 0)
            {
                AlignWithWall();
            }
        }
        else
        {
            // BluetoothSerial.println("Farming: Driving Backwards...");
            drive_tothis_poc(-REAR_WALL_TARGET); // Drives backward until US sensor reads (1980 - (210+10))mm

            // drive_tothis_poc(getUSDistance() - STARTING_US_DIST); // Drives backward until US sensor reads (1980 - (210+10))mm
            // BluetoothSerial.print("Driving backwards:");
            // BluetoothSerial.println(getUSDistance() - STARTING_US_DIST);
        }

        delay(75); // Allow momentum to settle

        // --- 2. CHECK IF WE REACHED THE END OF THE COURSE ---
        // Look at the LR sensor in the direction we are strafing.
        float side_dist = (strafe_dir == 1) ? getRightLR() : getLeftLR();
        if (side_dist < SIDE_WALL_THRESHOLD)
        {
            BluetoothSerial.println("Course fully traced. Farming complete.");
            course_completed = true;
            break;
        }

        // --- 3. STRAFE TO NEXT LANE ---
        BluetoothSerial.println("Farming: Strafing to next lane...");
        strafe_thismuch_poc(strafe_dir, LANE_WIDTH);

        // delay(300); // Allow momentum to settle before switching axis

        // Flip direction for the next trace
        driving_forward = !driving_forward;
    }

    stop();
}