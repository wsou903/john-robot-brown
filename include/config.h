#ifndef CONFIG_h
#define CONFIG_h

#include <Arduino.h>
#include <SoftwareSerial.h>  // Bluetooth output
#include <Adafruit_BNO08x.h> //Need for Gyroscope
#include <Servo.h>           // Servo library
#include <math.h>

// ------------- PIN DEFS
#define BLUETOOTH_RX 19
#define BLUETOOTH_TX 18
#define PIN_TRIG 48
#define PIN_ECHO 49

// IR Sensors
const int pinIR_Long1 = A4;  // GP2Y0A21YK0F (10-80cm)
const int pinIR_Long2 = A5;  // GP2Y0A21YK0F (10-80cm)
const int pinIR_Short1 = A6; // GP2Y0A41SK0F (4-30cm)
const int pinIR_Short2 = A7; // GP2Y0A41SK0F (4-30cm)

// Default motor control pins
// const byte left_front = 46;
// const byte left_rear = 47;
// const byte right_rear = 50;
// const byte right_front = 51;

const byte left_front = 50; // sevan: after checking with TAs on saturday we had to do this
const byte left_rear = 51;
const byte right_rear = 46;
const byte right_front = 47;

// ------------- CONSTANTS
#define TURNING_SAMPLES 57
#define JOHN_ROBOT_WIDTH 210
#define JOHN_ROBOT_LENGTH 210
#define WHEEL_RADIUS 2.75
#define HALF_LR_WHEEL_TO_WHEEL 9.5
#define HALF_FB_WHEEL_TO_WHEEL 8
#define FB_WHEEL_TO_WHEEL 16
#define US_OFFSET_SIDE 10 // 10cm from center to left side
#define US_OFFSET_FRONT 2 // 2cm from center to front
#define TABLE_WIDTH 121.5 // these probs change --------------------------------------
#define TABLE_HEIGHT 198.0
#define SR_SPACING 55.0
#define LR_TO_CENTER 6.5
#define US_TO_CENTER 10.0

// ------------ STATE MACHINE

enum STATE
{
  INITIALISING,
  RUNNING,
  STOPPED
};

// ------------ GLOBAL OBJECTS

extern SoftwareSerial BluetoothSerial;
extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;

extern Servo left_font_motor;
extern Servo left_rear_motor;
extern Servo right_rear_motor;
extern Servo right_font_motor;
extern Servo turret_motor;
// ------------ GLOBAL VARIABLES

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;
extern unsigned long last_gyro_time;
extern unsigned long lastTurretMove;
extern int FRspeed_val, FLspeed_val, BRspeed_val, BLspeed_val, speed_change, currentLane, turretInterval, turretStep, pos, speed_val;
extern float robotX, robotY, rad, gyro_bias, robot_heading, lastRightSR, lastLeftSR, lastRightLR, lastLeftLR, lastUS;

#endif
