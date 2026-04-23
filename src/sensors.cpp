
#include "sensors.h"
#include "kalmanfilter.h"

float lastRightSR = 100, lastLeftSR = 100;
float lastRightLR = 100, lastLeftLR = 100;
float lastUS = 100;
float alpha_SR = 0.1;
float alpha_LR = 0.1;
float alpha_US = 0.1;

// setup the kalman filter vars needs to be updated -------------------------------------------------------------------------------------------------
KalmanFilter kfUS(2.0, 2.0, 2);
KalmanFilter kfSR_L(2.0, 2.0, 5);
KalmanFilter kfSR_R(2.0, 2.0, 5);
KalmanFilter kfLR_L(5.0, 5.0, 1);
KalmanFilter kfLR_R(5.0, 5.0, 1);

// short range ir (front)
float getLeftSR() {

  float adcRaw = analogRead(pinIR_Short2);
  if(adcRaw == 0) adcRaw = 1;
  // float lastLeftSR = pow((adcRaw / 1562610.0), (1.0 / -1.98778)); //sevans calibration
  float lastLeftSR = pow((adcRaw / 31299.0), (1.0 / -1.067)); 
  // lastLeftSR = kfSR_L.updateEstimate(lastLeftSR); // IF KALMAN NOT CORRECTLY WORKING (DESPITE TESTING SHOWING IT WORKS) JUST REMOVE THIS LINE
  // float temp_val = 13*pow(adcRaw*0.0048828125, -1); // original calibration
  // lastLeftSR =  (alpha_SR * temp_val) + (1.0 - alpha_SR) * lastLeftSR;
  if (lastLeftSR > 200){
    lastLeftSR = 200;
  }
  return lastLeftSR;
}

float getRightSR() {
  float adcRaw = analogRead(pinIR_Short1);
  if (adcRaw == 0) adcRaw = 1;
  float lastRightSR = pow((adcRaw / 31299.0), (1.0 / -1.067));  // sevan calibration
  // lastRightSR = kfSR_L.updateEstimate(lastRightSR);  // IF KALMAN NOT CORRECTLY WORKING (DESPITE TESTING SHOWING IT WORKS) JUST REMOVE THIS LINE
   if (lastRightSR > 200){
    lastRightSR = 200;
  }
  return lastRightSR;
}


// long range ir (left)
float getLeftLR() { // outputs in fucking cm WTF

  float adcRaw = analogRead(pinIR_Long1);
  if (adcRaw == 0) adcRaw = 1;

  float voltage = adcRaw * (5.0 / 1023.0);
  float lastLeftLR = pow((voltage / 17.6), -1.144);
  lastLeftLR = lastLeftLR*10; // output mm?
  if (lastLeftLR > 800){
    lastLeftLR = 800;
  }
  // lastLeftLR = kfSR_L.updateEstimate(lastLeftLR); // IF KALMAN NOT CORRECTLY WORKING (DESPITE TESTING SHOWING IT WORKS) JUST REMOVE THIS LINE
  return lastLeftLR;
}

float getRightLR() {
  float adcRaw = analogRead(pinIR_Long2);
  if (adcRaw == 0) adcRaw = 1;

  float voltage = adcRaw * (5.0 / 1023.0);
  float lastRightLR = pow((voltage / 16.038), -1.210);
  lastRightLR = lastRightLR * 10;
  if (lastRightLR > 800){
    lastRightLR = 800;
  }
  // lastRightLR = kfSR_L.updateEstimate(lastRightLR); // IF KALMAN NOT CORRECTLY WORKING (DESPITE TESTING SHOWING IT WORKS) JUST REMOVE THIS LINE

  return lastRightLR;
}

// ultrasonic
float getUSDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH);
  float raw = (duration * 0.0343) / 2.0;
  //  lastUS =  (alpha_US * raw) + (1.0 - alpha_US) * lastUS;

  if (raw <= 0 || raw > 400)
  {
    return raw; // return last good global if out of range
  }

  // lastUS = kfUS.updateEstimate(raw); // IF KALMAN NOT CORRECTLY WORKING (DESPITE TESTING SHOWING IT WORKS) JUST REMOVE THIS LINE
  lastUS = raw; // JUST PUT THIS LINE IN

  return lastUS;  // return filtered value
}

void calibrateGyro() {
  float sum = 0;
  int samples = 100;

  for (int i = 0; i < samples; i++) {
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        sum += sensorValue.un.gyroscope.z;
      }
    }
    delay(5);
  }

  gyro_bias = sum / samples;
}

#ifndef NO_READ_GYRO
void GYRO_reading() {
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      unsigned long now = micros();
      float gyroZ = sensorValue.un.gyroscope.z;
      if (last_gyro_time == 0) {
        last_gyro_time = now;
      }  // First run fix
      float dt =
          (now - last_gyro_time) / 1000000.0;  // 1000000 to convert to seconds
      last_gyro_time = now;  // sets it to the current time, ready for the next
                             // time this function is called
      gyroZ -= gyro_bias;
      if (abs(gyroZ) < 0.02) {
        gyroZ = 0;
      }

      rad = gyroZ * dt;
      robot_heading += rad;

      if (robot_heading >= 6.28) robot_heading -= 6.28;
      if (robot_heading < 0) robot_heading += 6.28;

      // BluetoothSerial.println(rad);

    }
  }

  return;
}
#endif

float get_rotation_vector_yaw() {
  static float last_yaw = 0.0;
  static bool first_reading = true;
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      // Extract Quaternions
      float i = sensorValue.un.rotationVector.i;
      float j = sensorValue.un.rotationVector.j;
      float k = sensorValue.un.rotationVector.k;
      float r = sensorValue.un.rotationVector.real;

      // Conversion to Yaw (Heading) in Radians
      // Range is -PI to +PI
      last_yaw = atan2(2.0 * (r * k + i * j), 1.0 - 2.0 * (j * j + k * k));
      // BluetoothSerial.println(yaw);
      // delay(80);
      first_reading = false;
    }
  }
  if (first_reading) {
    return robot_heading;  // or some default value
  }
  robot_heading =
      last_yaw;     // Update global heading with the latest valid reading
  return last_yaw;  // Return the last valid yaw reading
}

void TestIRSensors() {
  static unsigned long timer = millis();

  while (true) {
    if ((millis() - timer) > 1000) {
      BluetoothSerial.print("right lr:");
      BluetoothSerial.println(getRightLR());
      delay(500);
      BluetoothSerial.print(",");
      BluetoothSerial.print("left lr:");
      BluetoothSerial.println(getLeftLR());
      delay(500);
      // Serial.print("LR1:");
      // Serial.print(getLeftLR());
      // Serial.print(",");
      // Serial.print("LR2:");
      // Serial.print(getRightLR());
      // BluetoothSerial.print("US:");
      // BluetoothSerial.println(getUSDistance());

      BluetoothSerial.println();
      delay(50);

      timer = millis();
    }
  }
}