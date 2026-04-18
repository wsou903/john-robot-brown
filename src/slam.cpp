

#include "slam.h"

// states for servo sweeping with the ultrasonic sensor
enum US_STATE
{
    POINT_FRONT,
    WAIT_FRONT,
    READ_FRONT,
    POINT_LEFT,
    WAIT_LEFT,
    READ_LEFT,
    POINT_BACK,
    WAIT_BACK,
    READ_BACK
};

US_STATE us_state = POINT_FRONT;
unsigned long us_timer = 0;
const int SERVO_DELAY = 300; // ms to allow servo to physically move

// latest US readings
float latest_us_front = -1;
float latest_us_left = -1;
float latest_us_back = -1;

void init_slam()
{
    // start pointing forward, think thats 0 degree? need to check
    turret_motor.write(0);
}

// helper functions to set sensor limits
float getValidSR(float raw)
{
    if (raw >= 4.0 && raw <= 30.0)
    {
        return raw;
    }
    return -1.0;
}

float getValidLR(float raw)
{
    if (raw >= 10.0 && raw <= 80.0)
    {
        return raw;
    }
    return -1.0;
}

void budget_slam()
{
    unsigned long now = millis();

    // no blockinggggg
    switch (us_state)
    {
    case POINT_FRONT:
        turret_motor.write(0); // front
        us_timer = now;
        us_state = WAIT_FRONT;
        break;
    case WAIT_FRONT:
        if (now - us_timer >= SERVO_DELAY)
            us_state = READ_FRONT;
        break;
    case READ_FRONT:
        latest_us_front = getUSDistance();
        us_state = POINT_LEFT;
        break;

    case POINT_LEFT:
        turret_motor.write(90); // left
        us_timer = now;
        us_state = WAIT_LEFT;
        break;
    case WAIT_LEFT:
        if (now - us_timer >= SERVO_DELAY)
            us_state = READ_LEFT;
        break;
    case READ_LEFT:
        latest_us_left = getUSDistance();
        us_state = POINT_BACK;
        break;

    case POINT_BACK:
        turret_motor.write(180); // back
        us_timer = now;
        us_state = WAIT_BACK;
        break;
    case WAIT_BACK:
        if (now - us_timer >= SERVO_DELAY)
            us_state = READ_BACK;
        break;
    case READ_BACK:
        latest_us_back = getUSDistance();
        us_state = POINT_FRONT; // go back and start again, maybe change ? ------------------------------------
        break;
    }

    // reading sr sensors (front)
    float sr_left = getValidSR(getLeftSR());
    float sr_right = getValidSR(getRightSR());

    // reading lr sensors (right)
    float lr_front = getValidLR(getRightLR());
    float lr_back = getValidLR(getLeftLR());

    // for the helper addCoordEst
    int x_count = 0;
    int y_count = 0;
    float sum_x = 0;
    float sum_y = 0;

    // uses the angle of the robot (and servo) to work out the accurate dist values
    auto addCoordinateEstimate = [&](float distance, float mounted_angle)
    {
        if (distance < 0)
        {
            return; // ignores the out of range sensors
        }

        // this is the robot angle + the servo turned angle (mounted angle will be 0 for ir sensors)
        float global_angle = robot_heading + mounted_angle;

        // warp angle to stay within -PI and +PI
        while (global_angle > PI)
            global_angle -= TWO_PI;
        while (global_angle < -PI)
            global_angle += TWO_PI;

        // calc x axis, if cos(angle) is > 0.707 or < -0.707, it is pointing mostly forward/backward
        if (abs(cos(global_angle)) > 0.707)
        {
            if (cos(global_angle) > 0)
            {
                // pointing front and taking away the dist from the total to work out where we are
                // cos(angle) * hypotenuse = adjacent
                sum_x += TABLE_HEIGHT - (distance * abs(cos(global_angle)));
            }
            else
            {

                sum_x += distance * abs(cos(global_angle));
            }
            x_count++;
        }

        // calc y axis, if sin(angle) is > 0.707 or < -0.707, it is pointing mostly sideways
        if (abs(sin(global_angle)) > 0.707)
        {
            if (sin(global_angle) > 0)
            {
                // sin(angle) * hypotenuse = y
                sum_y += distance * abs(sin(global_angle));
            }
            else
            {
                // the opposite taking away the table width to make all the same
                sum_y += TABLE_WIDTH - (distance * abs(sin(global_angle)));
            }
            y_count++;
        }
    };

    // RADIANSSSSSSSSSSSSSS
    // front = 0, left = pi/2 (1.57), back = pi (3.14)

    // sr ir
    addCoordinateEstimate(sr_left, 0.0);
    addCoordinateEstimate(sr_right, 0.0);

    // lr ir
    addCoordinateEstimate(lr_front, -HALF_PI);
    addCoordinateEstimate(lr_back, -HALF_PI);

    // ultrasonic front left right
    addCoordinateEstimate(latest_us_front, 0.0);
    addCoordinateEstimate(latest_us_left, HALF_PI);
    addCoordinateEstimate(latest_us_back, PI);

    // 4. AVERAGE THE VALID ESTIMATES
    if (x_count > 0)
        robotX = sum_x / x_count;
    if (y_count > 0)
        robotY = sum_y / y_count;

    // 5. OUTPUT TO BLUETOOTH (Rate limited to ~10Hz to prevent buffer crashing)
    static unsigned long print_timer = 0;
    if (now - print_timer > 100)
    {
        BluetoothSerial.print("DATA,");
        BluetoothSerial.print(robotX);
        BluetoothSerial.print(",");
        BluetoothSerial.print(robotY);
        BluetoothSerial.print(",");
        BluetoothSerial.println(robot_heading); // Sending heading is highly useful for mapping!
        print_timer = now;
    }
}

// void budget_SLAM(float USdistances[], float LeftIRReadings[], float RightIRReadings[], float angles[], int count)
// {

//     // find closest wall
//     int closest_wall_index_rough = FindMinIndex(USdistances, count);

//     // check with irs first (in case we are in middle)
//     float checkIR = (LeftIRReadings[closest_wall_index_rough] + RightIRReadings[closest_wall_index_rough]) / 2.0;

//     // default to closest wall (US reading)
//     int closest_wall_index = closest_wall_index_rough;

//     if (checkIR > 5.0 && checkIR < 75.0)
//     {
//         float best_IR_sum = 999.0; // arbitrary num

//         for (int j = -5; j <= 5; j++)
//         {
//             int check_idx = (closest_wall_index_rough + j + count) % count;
//             // seeing the diff between front ir sensors
//             float current_IR_sum = LeftIRReadings[check_idx] + RightIRReadings[check_idx];

//             if (current_IR_sum < best_IR_sum)
//             {
//                 best_IR_sum = current_IR_sum;
//                 closest_wall_index = check_idx;
//             }
//         }
//     }
//     else
//     {

//         BluetoothSerial.println("you are in the fucking middle you lunatic");
//     }

//     // get the other 3 walls (90 degrees apart)
//     int wall_two_index = (closest_wall_index + (count / 4)) % count;
//     int wall_three_index = (closest_wall_index + (count / 2)) % count;
//     int wall_four_index = (closest_wall_index + (3 * count / 4)) % count;

//     // calculate the distances
//     float distN = USdistances[closest_wall_index];
//     float distW = USdistances[wall_two_index];
//     float distS = USdistances[wall_three_index];
//     float distE = USdistances[wall_four_index];

//     // calculate length and width of the table (should be 200 x 100?) robot size declared at top of code, need to update
//     float roomWidthCalc = distW + distE + JOHN_ROBOT_WIDTH;
//     float roomHeightCalc = distN + distS + JOHN_ROBOT_LENGTH;

//     // calculate the x and y coord based on the bottom left corner (south and west)
//     robotX = distW + (JOHN_ROBOT_WIDTH / 2.0);
//     robotY = distS + (JOHN_ROBOT_LENGTH / 2.0);

//     // --- Output Results to Bluetooth ---
//     BluetoothSerial.println("--- SLAM LOCALIZATION ---");
//     BluetoothSerial.print("DATA,");
//     BluetoothSerial.print(robotX);
//     BluetoothSerial.print(",");
//     BluetoothSerial.print(robotY);
//     BluetoothSerial.print(",");
//     BluetoothSerial.print(roomWidthCalc);
//     BluetoothSerial.print(",");
//     BluetoothSerial.println(roomHeightCalc);
// }