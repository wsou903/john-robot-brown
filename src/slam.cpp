#include "slam.h"
#include "helpers.h"
#include "sensors.h"

const int pos_readings = 500; // 500 points at 100ms = 50 seconds of recording
float history_X[pos_readings];
float history_Y[pos_readings];
int slam_point_count = 0;

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
    float sr_front_left = getValidSR(getLeftSR());
    float sr_front_right = getValidSR(getRightSR());
    float lr_left = getValidLR(getLeftLR());   // Assuming Left LR is on Left side
    float lr_right = getValidLR(getRightLR()); // Assuming Right LR is on Right side

    int x_count = 0;
    int y_count = 0;
    float sum_x = 0;
    float sum_y = 0;

    // get coord estimate using lambda func
    auto addCoordinateEstimate = [&](float distance, float mounted_angle)
    {
        if (distance <= 0)
            return; // Ignore invalid readings

        // Robot orientation + where the sensor is pointing
        float global_angle = robot_heading + mounted_angle;

        // Normalize angle to -PI to PI
        while (global_angle > PI)
            global_angle -= TWO_PI;
        while (global_angle < -PI)
            global_angle += TWO_PI;

        float c = cos(global_angle);
        float s = sin(global_angle);

        // Calculate X (Pointing toward Front/Back walls)
        if (abs(c) > 0.707) // Within 45 degrees of X axis
        {
            if (c > 0)
                sum_x += TABLE_LENGTH - (distance * abs(c));
            else
                sum_x += (distance * abs(c));
            x_count++;
        }

        // Calculate Y (Pointing toward Side walls)
        if (abs(s) > 0.707) // Within 45 degrees of Y axis
        {
            if (s > 0)
                sum_y += distance * abs(s);
            else
                sum_y += TABLE_WIDTH - (distance * abs(s));
            y_count++;
        }
    };

    // map angles
    // Front sensors (angle 0)
    addCoordinateEstimate(us_front, 0.0);
    addCoordinateEstimate(sr_front_left, 0.0);
    addCoordinateEstimate(sr_front_right, 0.0);

    // Side sensors (Left is +PI/2, Right is -PI/2)
    addCoordinateEstimate(lr_left, HALF_PI);   // 90 degrees
    addCoordinateEstimate(lr_right, -HALF_PI); // -90 degrees

    // average the results
    if (x_count > 0)
        robotX = sum_x / x_count;
    if (y_count > 0)
        robotY = sum_y / y_count;
    BluetoothSerial.print(robotX);
    BluetoothSerial.print(",");
    BluetoothSerial.println(robotY);
}

// void dump_slam_data()
// {

//     BluetoothSerial.println("X, Y"); // CSV Header

//     for (int i = 0; i < slam_point_count; i++)
//     {
//         SerialCom->println(history_X[i]);
//         SerialCom->println(",");
//         SerialCom->println(history_Y[i]);
//         BluetoothSerial.print(history_X[i]);
//         BluetoothSerial.print(",");
//         BluetoothSerial.print(history_Y[i]);
//     }

//     BluetoothSerial.println("end");

//     // Optional: Reset the count if you plan to run the robot again without resetting power
//     // slam_point_count = 0;
// }