#include "slam.h"
#include "helpers.h"
#include "sensors.h"
#include "farming.h"
#include "homing.h"

const int pos_readings = 200; // 500 points at 100ms = 50 seconds of recording
int history_X[pos_readings];
int history_Y[pos_readings];
int slam_point_count = 0;

int direction = 0;

// helper functions to set sensor limits
float getValidSR(float raw)
{
    return (raw >= 4.0 && raw <= 30.0) ? raw : -1.0;
}

float getValidLR(float raw)
{
    return (raw >= 10.0 && raw <= 80.0) ? raw : -1.0;
}

void budget_slam()
{
    unsigned long now = millis();

    // get all sensors values
    float us_front = getUSDistance();
    // float sr_front_left = getValidSR(getLeftSR());
    // float sr_front_right = getValidSR(getRightSR());
    // float lr_left = getValidLR(getLeftLR());   // Assuming Left LR is on Left side
    // float lr_right = getValidLR(getRightLR()); // Assuming Right LR is on Right side
    float lr_left = getLeftLR();
    float lr_right = getRightLR();
    float yaw = get_rotation_vector_yaw();

    float x_count = 0;
    float y_count = 0;
    float sum_x = 0;
    float sum_y = 0;

    // get coord estimate using lambda func
    // auto addCoordinateEstimate = [&](int distance, float mounted_angle)
    // {
    //     if (distance <= 0)
    //         return; // Ignore invalid readings

    //     // Robot orientation + where the sensor is pointing
    //     float global_angle = robot_heading + mounted_angle;

    //     // Normalize angle to -PI to PI
    //     while (global_angle > PI)
    //         global_angle -= TWO_PI;
    //     while (global_angle < -PI)
    //         global_angle += TWO_PI;

    //     float c = cos(global_angle);
    //     float s = sin(global_angle);

    //     // Calculate X (Pointing toward Front/Back walls)
    //     if (abs(c) > 0.707) // Within 45 degrees of X axis
    //     {
    //         if (c > 0)
    //             sum_x += TABLE_LENGTH - (distance * abs(c));
    //         else
    //             sum_x += (distance * abs(c));
    //         x_count++;
    //     }

    //     // Calculate Y (Pointing toward Side walls)
    //     if (abs(s) > 0.707) // Within 45 degrees of Y axis
    //     {
    //         if (s > 0)
    //             sum_y += distance * abs(s);
    //         else
    //             sum_y += TABLE_WIDTH - (distance * abs(s));
    //         y_count++;
    //     }
    // };
    //   int sum_y = 0;
    // int y_count = 0;
     // bool sensor_in_range = (getLeftLR() > getRightLR());

    // if (direction && sensor_in_range)
    // {
    //     robotY = robotY + lr_right;
    // }
    // else if (direction && !sensor_in_range)
    // {
    //     robotY = robotY + lr_left;
    // }
    // else if (!direction && sensor_in_range)
    // {
    //     robotY = robotY + lr_right;
    // }
    // else
    // {
    //     robotY = robotY + lr_left;
    // }
    // 3. Calculate Y from Left Sensor(if valid) 
    // if (lr_left > 0)
    // {
    //     // We use cos(robot_heading) to correct for the robot being tilted
    //     float y_estimate_left = (lr_left * cos(yaw)) + (JOHN_ROBOT_WIDTH / 2);
    //     sum_y += y_estimate_left;
    //     y_count++;
    // }

    // // 4. Calculate Y from Right Sensor (if valid)
    // if (lr_right > 0)
    // {
    //     // Distance from right wall subtracted from total width
    //     float y_estimate_right = (TABLE_WIDTH - ((lr_right * cos(yaw)) + (JOHN_ROBOT_WIDTH / 2)));
    //     sum_y += y_estimate_right;
    //     y_count++;
    // }

    // // 5. Average the Y results
    // if (y_count > 0)
    // {
    //     robotY = (sum_y / y_count);
    // }
    // bool sensor_in_range = (getLeftLR() > getRightLR());

    // if (direction && sensor_in_range)
    // {
    //     robotY = lr_right;
    // }
    // else if (direction && !sensor_in_range)
    // {
    //     robotY = robotY + lr_left;
    // }
    // else if (!direction && sensor_in_range)
    // {
    //     robotY = robotY + lr_right;
    // }
    // else
    // {
    //     robotY = robotY + lr_left;
    // }

    // 1. Check which sensor is closer (smaller value is usually more reliable)
    // bool left_is_closer = (lr_left < lr_right);

    // if (left_is_closer)
    // {
    //     // Y is just the distance from the left wall (0)
    //     // Plus the offset for the robot's center
    //     robotY = lr_left + (JOHN_ROBOT_WIDTH / 2.0);
    // }
    // else 
    // {
    //     // Y is the Table Width minus the distance from the right wall
    //     // This flips the coordinate so both sensors agree on the same Y-plane
    //     robotY = TABLE_WIDTH - (lr_right + (JOHN_ROBOT_WIDTH / 2.0));
    // }

    if (strafe_dir_global == 0){
        robotY = lr_left + (JOHN_ROBOT_WIDTH / 2.0);
    } else {
        robotY = TABLE_WIDTH - (lr_right + (JOHN_ROBOT_WIDTH / 2.0));
    }
    // robotY = lr_left + (JOHN_ROBOT_WIDTH / 2.0);

    robotX = (us_front + ((JOHN_ROBOT_WIDTH / 10) / 2))*10;


    // map angles
    // Front sensors (angle 0)
    // addCoordinateEstimate(us_front, 0.0);
    // addCoordinateEstimate(sr_front_left, 0.0);
    // addCoordinateEstimate(sr_front_right, 0.0);

    // // Side sensors (Left is +PI/2, Right is -PI/2)
    // addCoordinateEstimate(lr_left, HALF_PI);   // 90 degrees
    // addCoordinateEstimate(lr_right, -HALF_PI); // -90 degrees

    // average the results
    if (x_count > 0)
        robotX = sum_x / x_count;
    if (y_count > 0)
        robotY = sum_y / y_count;

    // save data to arrays (10Hz)
    // static unsigned long save_timer = 0;
    // if (now - save_timer > 250)
    // {
    //     if (slam_point_count < pos_readings)
    //     {
    //         history_X[slam_point_count] = robotX;
    //         history_Y[slam_point_count] = robotY;
    //         slam_point_count++;
    //     }
    //     else
    //     {

    //         // BluetoothSerial.println("slam full");
    //     }
    //     save_timer = now;
    // }
    // 5. OUTPUT DATA

    robot_heading_global = robot_heading_global + get_rotation_vector_yaw();

    static unsigned long print_timer = 0;
    if (now - print_timer > 100)
    {
        BluetoothSerial.print("DATA,");

        BluetoothSerial.print(robotX);

        BluetoothSerial.print(",");

        BluetoothSerial.print(robotY);

        BluetoothSerial.print(",");

        BluetoothSerial.println(robot_heading_global * 180.0 / PI); // Deg for easier reading
        print_timer = now;
    }
}

// void budget_slam()
// {
//     unsigned long now = millis();

//     // get all sensors values
//     float us_front = getUSDistance();
//     float lr_left = getValidLR(getLeftLR());   // Assuming Left LR is on Left side
//     float lr_right = getValidLR(getRightLR()); // Assuming Right LR is on Right side
//     float yaw = get_rotation_vector_yaw();

//     // bool sensor_in_range = (getLeftLR() > getRightLR());

//     // if (direction && sensor_in_range)
//     // {
//     //     robotY = robotY + lr_right;
//     // }
//     // else if (direction && !sensor_in_range)
//     // {
//     //     robotY = robotY + lr_left;
//     // }
//     // else if (!direction && sensor_in_range)
//     // {
//     //     robotY = robotY + lr_right;
//     // }
//     // else
//     // {
//     //     robotY = robotY + lr_left;
//     // }

//     int sum_y = 0;
//     int y_count = 0;

//     // 3. Calculate Y from Left Sensor(if valid) if (lr_left > 0)
//     {
//         // We use cos(robot_heading) to correct for the robot being tilted
//         int y_estimate_left = (lr_left * cos(yaw)) + (JOHN_ROBOT_WIDTH / 2);
//         sum_y += y_estimate_left;
//         y_count++;
//     }

//     // 4. Calculate Y from Right Sensor (if valid)
//     if (lr_right > 0)
//     {
//         // Distance from right wall subtracted from total width
//         float y_estimate_right = (TABLE_WIDTH - ((lr_right * cos(yaw)) + (JOHN_ROBOT_WIDTH / 2)));
//         sum_y += y_estimate_right;
//         y_count++;
//     }

//     // 5. Average the Y results
//     if (y_count > 0)
//     {
//         robotY = (sum_y / y_count);
//     }

//     robotX = us_front + ((JOHN_ROBOT_WIDTH / 10) / 2);

//     static unsigned long print_timer = 0;
//     if (now - print_timer > 250)
//     {
//         BluetoothSerial.print("DATA,");
//         delay(10);
//         BluetoothSerial.print(robotX);
//         delay(10);
//         BluetoothSerial.print(",");
//         delay(10);
//         BluetoothSerial.print(robotY);
//         delay(10);
//         BluetoothSerial.print(",");
//         delay(10);
//         BluetoothSerial.println(robot_heading * 180.0 / PI); // Deg for easier reading
//         print_timer = now;
//     }
// }

// void dump_slam_data()
// {
//     BluetoothSerial.println("--- DUMP START ---");
//     delay(10);
//     BluetoothSerial.print("Total Points Recorded: ");
//     delay(10);
//     BluetoothSerial.println(slam_point_count);
//     delay(10);
//     BluetoothSerial.println("X, Y"); // CSV Header
//     delay(10);
//     for (int i = 0; i < slam_point_count; i++)
//     {
//         BluetoothSerial.print(history_X[i]);
//         delay(10);
//         BluetoothSerial.print(",");
//         delay(10);
//         BluetoothSerial.println(history_Y[i]);
//         delay(10);
//     }

//     BluetoothSerial.println("end");
// }