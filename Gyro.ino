
void readGyro() {
  int gyroPin = A12;               //Gyro is connected to analog pin 19
  float gyroVoltage = 3.3;         //Gyro is running at 3.3V
  //float gyroSensitivity = .002675;
  float rotationThreshold = 5;   //Minimum deg/sec to keep track of - helps with gyro drifting
  //This line converts the 0-1023 signal to 0-3.3V
  float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 4096.0;
  //This line finds the voltage offset from sitting still
  gyroRate -= gyroZeroVoltage;
  //Serial.println(gyroRate);
  //This line divides the voltage we found by the gyro's sensitivity
  if (moveType == TURN_RIGHT) {
//    gyroSensitivity = .00265;//TODO Not sure about this
  }
  gyroRate /= gyroSensitivity;

  //Ignore the gyro if our angular velocity does not meet our threshold
  if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    gyroRate /= 1000.0;
    angle += gyroRate;
  }
}

void setupGyro() {
  for (int i = 0; i < 1000; i++) {
    gyroZeroVoltage+= (analogRead(A12)*3.3/4096);
    delayMicroseconds(100);
  }
  gyroZeroVoltage /= 1000;
}

void readEncoderAngle() {
  const float wheelDist = 60;//mm
//  const float wheelDist = 68.73;//mm
//  const float wheelDist = 79.0;//mm
  float encoderRate = (leftSpeed - rightSpeed) / wheelDist;
  encoderRate /= 20;
  if (encoderRate < 0) {
    encoderRate *= 1;
  }
  encoderAngle += encoderRate;
}


