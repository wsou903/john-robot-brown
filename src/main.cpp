// #define NO_READ_GYRO  //Uncomment if GYRO is not attached.
// #define NO_HC -SR04  //Uncomment if HC-SR04 ultrasonic ranging sensor is not attached.
// #define NO_BATTERY_V_OK //Uncomment if BATTERY_V_OK if you do not care about battery damage.

#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "helpers.h"
#include "homing.h"
#include "slam.h"
#include "farming.h"

// Gyroscope initialisation - Define your reset pin if you have it connected, otherwise use -1
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);
HardwareSerial *SerialCom;

Servo left_font_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_font_motor;

float robot_heading = 0.0, gyro_bias = 0.0, rad = 0.0;
unsigned long last_gyro_time = 0;
float robotX = 0.0, robotY = 0.0;

// motor speed vars
int speed_val = 150;  // starting speed
int speed_change = 0; // used for ramping speed up/down
int FRspeed_val = 0;
int FLspeed_val = 0;
int BRspeed_val = 0;
int BLspeed_val = 0;

Servo turret_motor;
//   int pos = 90;
//   int turretStep = 1;          // How many degrees to move each time
//    int turretInterval = 50;
// unsigned long lastTurretMove = 0;

STATE initialising();
STATE running();
STATE stopped();
boolean is_battery_voltage_OK();
//----------------------------------------------------------------------------------------------------------------¿

void setup()
{
  delay(10);
  SerialCom = &Serial;
  SerialCom->begin(115200);
  BluetoothSerial.begin(115200);

  // Setup Ultrasonic Pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Turret Setup
  turret_motor.attach(9);
}

void loop()
{
  static STATE machine_state = INITIALISING;
  if (machine_state == RUNNING)
  {
    budget_slam();
  }
  // Finite-state machine Code
  switch (machine_state)
  {
  case INITIALISING:
    machine_state = initialising();
    break;
  case RUNNING: // Lipo Battery Volage OK
    machine_state = running();
    break;
  case STOPPED: // Stop of Lipo Battery voltage is too low, to protect Battery
    machine_state = stopped();
    break;
  };
}

// Function defs

// STATES ---------------------------------------------------------------------------------------------------------------------------------------
STATE initialising()
{
  // initialising
  SerialCom->println("INITIALISING....");
  BluetoothSerial.println("INITIALISING....");

  enable_motors();
#ifndef NO_READ_GYRO
  SerialCom->println("Enabling Gyroscope...");
  if (!bno08x.begin_I2C() || !bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000))
  {
    while (1)
    {
      BluetoothSerial.println("IMU failed");
      delay(100);
    }
  }
#endif

  calibrateGyro(); // This is where the gyro calibration is done //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  delay(3000);
  G28(); // homing to get to corner ////////////////////////////////// G28 :)
  SerialCom->println("RUNNING STATE...");
  BluetoothSerial.println("RUNNING STATE...");

  return RUNNING;
}

STATE running()
{

  static unsigned long previous_millis;

  // read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500)
  { // Arduino style 500ms timed execution statement
    previous_millis = millis();

    // delay(30000);
    // budget_slam();
    // TestIRSensors();
    // strafe_thismuch_poc(1, 100); // Strafe right for 100mm
    // delay(5000);

    // drive_tothis_poc_GV(-200); // Drive forward for 200mm

    farming();
    // BluetoothSerial.println("course completion");
    // strafe_straight_poc(1); // Strafe right until wall proximity
    dump_slam_data();
    delay(30000);

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK())
      return STOPPED;
#endif
  }

  return RUNNING;
}

// Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped()
{
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500)
  { // print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");

#ifndef NO_BATTERY_V_OK
    // 500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK())
    {
      // SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      // SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10)
      { // Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        // SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    }
    else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

// END STATES ----------------------------------------------------------------------------------------------------------

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{ // Checking if the battery is ok and what the charge on the battery is
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  // the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  // to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  // Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160)
  {
    previous_millis = millis();
    // SerialCom->print("Lipo level:");
    // SerialCom->print(Lipo_level_cal);
    // SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    // SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  }
  else
  {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else
    {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif

// // Reading inputs from the keyboard -------------------------------------------------------------------------
// // Serial command pasing
// void read_serial_command()
// {
//   if (SerialCom->available())
//   {
//     char val = SerialCom->read();
//     SerialCom->print("Speed:");
//     SerialCom->print(speed_val);
//     SerialCom->print(" ms ");

//     // Perform an action depending on the command
//     switch (val)
//     {
//     case 'w': // Move Forward
//     case 'W':
//       forward();
//       SerialCom->println("Forward");
//       break;
//     case 's': // Move Backwards
//     case 'S':
//       reverse();
//       SerialCom->println("Backwards");
//       break;
//     case 'q': // Turn Left
//     case 'Q':
//       strafe_left();
//       SerialCom->println("Strafe Left");
//       break;
//     case 'e': // Turn Right
//     case 'E':
//       strafe_right();
//       SerialCom->println("Strafe Right");
//       break;
//     case 'a': // Turn Right
//     case 'A':
//       ccw();
//       SerialCom->println("ccw");
//       break;
//     case 'd': // Turn Right
//     case 'D':
//       cw();
//       SerialCom->println("cw");
//       break;
//     case '-': // Turn Right
//     case '_':
//       speed_change = -100;
//       SerialCom->println("-100");
//       break;
//     case '=':
//     case '+':
//       speed_change = 100;
//       SerialCom->println("+");
//       break;
//     case 'g':
//     case 'G':
//       ReadUSSensor();
//       SerialCom->println("Reading US Sensor");
//       break;
//     case 'y':
//     case 'Y':
//       ReadIRSensors();
//       SerialCom->println("Reading IR Sensors");
//       break;
//     default:
//       stop();
//       SerialCom->println("stop");
//       break;
//     }
//   }
// }
