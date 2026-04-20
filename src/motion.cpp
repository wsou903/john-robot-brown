#include "motion.h"
#include "helpers.cpp"
#include "sensors.cpp"
#include "motors.cpp"
#include "config.h"

void strafe_straight_poc(){

  int ir_enabled = 0;
  int us_enabled = 1;
  int gyro_enabled = 1;
  int derivative_enabled = 0;
  // lk its fine without the D term with just PI 120/3

  float last_print = millis();

  bool wall_proximity = false;

  // PID VALUES
  float kp_ir = 0.5*ir_enabled;
  float kp_gyro = 120*5*gyro_enabled;
  float kp_us = 15*us_enabled;

  float ki_ir = 0.001*ir_enabled;
  float ki_gyro = 3*gyro_enabled;
  float ki_us = 0.01*us_enabled;

  float kd_gyro = 5*derivative_enabled;

  float err_ir;
  float err_gyro;
  float err_us;

  float ir_u;
  float gyro_u;
  float us_u;

  float gyro_read;
  float avg_lr_read;
  float us_read;

  // sets the current wall distance to be the r(t)
  // sets current gyro to become the reference angle
  float lr_initial = avg_lr_read;
  gyro_read = get_rotation_vector_yaw();
  float gyro_initial = gyro_read;
  ReadUSSensor();
  us_read = distanceUS;
  float us_initial = us_read;


  // loop
  while (!wall_proximity){
    
    ReadIRSensors();
    avg_lr_read = (distLR1 + distLR2) / 2.0;
    gyro_read = get_rotation_vector_yaw();

    // Short range IR sensor outputs Right and Left
    float sr_right = pow((adcRaw3 / 31299.0), (1.0 / -1.067));
    float sr_left = pow((adcRaw4 / 1562610.0), (1.0 / -1.98778));

    ReadUSSensor();
    us_read = distanceUS;

    // BluetoothSerial.print("lr1 dist: ");
    // BluetoothSerial.println(distLR1);
    // BluetoothSerial.print("lr2 dist: ");
    // BluetoothSerial.println(distLR2);


    // STOP CONDITION
    if (sr_left < 60 || sr_right < 60) {
      wall_proximity = true;
      stop();
      delay(2000);
      break;
    }

    // error calcs
    err_ir = lr_initial - avg_lr_read;
    err_gyro = angle_diff(gyro_initial, gyro_read);
    err_us = us_initial - us_read;

    // // deadband for stopping the error if its too low // removing for now because idk if its needed 
    // if (abs(err_ir) < 1) err_ir = 0;
    // if (abs(err_gyro) < 0.5) err_gyro = 0;

    float d_err = err_gyro - prev_err_gyro;
    prev_err_gyro = err_gyro;

    // integral terms and windup prevention
    integral_sum_ir += err_ir; 
    integral_sum_gyro += err_gyro;
    integral_sum_us += err_us;
    float int_clamp = 100;
    integral_sum_ir = constrain(integral_sum_ir, -int_clamp, int_clamp);
    integral_sum_gyro = constrain(integral_sum_gyro, -int_clamp, int_clamp);
    integral_sum_us = constrain(integral_sum_us, -int_clamp, int_clamp);


    // control effort calcs
    ir_u = kp_ir *  err_ir + ki_ir * integral_sum_ir;
    gyro_u = kp_gyro * err_gyro + ki_gyro * integral_sum_gyro + kd_gyro * d_err;
    us_u = kp_us * err_us; // + ki_us * integral_sum_us;

    // // clamping?? idk
    // gyro_u = constrain(gyro_u, -80, 80);

    left_font_motor.writeMicroseconds(  1500 + speed_val - gyro_u  - us_u);
    left_rear_motor.writeMicroseconds(  1500 - speed_val - gyro_u  - us_u);
    right_font_motor.writeMicroseconds( 1500 - speed_val - gyro_u  + us_u);
    right_rear_motor.writeMicroseconds( 1500 + speed_val - gyro_u  + us_u);

    // DEBUGS 
    if (millis() - last_print > 100) {
      // BluetoothSerial.print("err_gyro: ");
      // BluetoothSerial.println(err_gyro, 4);
      // BluetoothSerial.println();
      BluetoothSerial.print("us_u: ");
      BluetoothSerial.println(us_u, 2);
      BluetoothSerial.println();
      last_print = millis();
    }


    delay(10); // DELAY ///////////////
  }
        
  stop();
  BluetoothSerial.println("YAYAYAYAY");
}