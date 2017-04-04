//void turnCorrection() {
//  static double oldErrorP;
//  double errorD;
//  double totalError;
//  double Kp = 10;
//  //double Kd = 10;
//  double targetAngle;
//  //long long start = millis();
//  //int timeConst = 2 * turnSpeed; //ms 2
//  static int i = 0;
//  static bool continueTurn = true;
//  static bool turn = false;
//  static bool straight = false;
//  static bool wallInFront = false;
//  accelerate = false;
//  if (i == 0) {
//    gyroSensitivity = .0037;
//    myDisplay.clear();
//    angle = 0;
//    encoderAngle = 0;
//    //    myDisplay.setCursor(0);
//    //    myDisplay.clear();
//    //    if (moveType == TURN_RIGHT) {
//    //      //      myDisplay.print("RGHT");
//    //    }
//    //    else {
//    //      //      myDisplay.print("LEFT");
//    //    }
//  }
//  if (turn == false) {
//    if (wallFront()) {
//      wallInFront = true;
//      targetAngle = 0;// TODO do forward correction during this part?
//      if ((rightFront + leftFront) / 2 > frontStop) {
//        turn = true;
//        i = 1;
//        //        i = round(pow(11.346 * abs(angleConst*(rightFront - leftFront)),0.5492));//how does R-L relate to angle?
//        //                angle = 1*(rightFrontRaw - leftFrontRaw);
//        //        angle = 0.0;
//      }
//    }
//    else {
//      //      myDisplay.setCursor(0);
//      //      myDisplay.clear();
//      //      myDisplay.print("NoWF");
//      if ((leftTicks + rightTicks) / 2 >= 1500) {//TODO Find this value, add correction
//        //        if ((moveType == TURN_RIGHT && !wallRight()) || moveType == TURN_LEFT && !wallLeft()) {
//        turn = true;
//        i = 1;
//        //          angle = 0.0;
//        //        }
//      }
//    }
//    //turn = true;
//  }
//  else {
//    if (moveType == TURN_RIGHT) {
//      targetAngle = curve5[i];
//    }
//    else {
//      targetAngle = -curve5[i];
//    }
//  }
//  if (moveType == TURN_RIGHT) {
//    //    if (straight == false && rightMiddleValue > 1250) {
//    //      angle = 90;
//    //      encoderAngle = 90;
//    //      targetAngle = 90;
//    //    }
//  }
//  if (moveType == TURN_LEFT) {
//    //    if (straight == false && leftMiddleValue > 1250) {
//    //      angle = -90;
//    //      encoderAngle = -90;
//    //      targetAngle = -90;
//    //    }
//  }
//
//  if (straight == false && abs(targetAngle) == 90) {
//    prevRightTicks -= rightTicks;
//    prevLeftTicks -= leftTicks;
//    rightTicks = 0;
//    leftTicks = 0;
//    straight = true;
//  }
//  if (straight == true) {
//    if (moveType == TURN_RIGHT) {
//      targetAngle = 90;
//    }
//    else {
//      targetAngle = -90;
//    }
//    if ((rightTicks + leftTicks) / 2 > 1500  || (leftFront + rightFront) / 2 > frontStop) {//TODO CHECK THIS TICK VALUE
//      continueTurn = false;
//    }
//  }
//
//  if ((targetAngle == 0 || abs(targetAngle) == 90) && wallFront()) {
//    errorP = .02 * (rightFrontRaw - leftFrontRaw) + targetAngle - encoderAngle;
//    //    errorP = 2 * (rightFront - leftFront);
//    //        errorP = targetAngle - angle;
//    //        errorP = .5 * (rightFront - leftFront - 0.0) + targetAngle - angle;
//  }
//  else {
//    errorP = targetAngle - encoderAngle;
//  }
//  errorD = errorP - oldErrorP;
//  totalError = 18 * errorP + 10 * errorD;
//  // Calculate PWM based on Error
//  currentLeftPWM = leftBaseSpeed + int(totalError);
//  currentRightPWM = rightBaseSpeed - int(totalError);
//  //    myDisplay.setCursor(0);
//  //    myDisplay.clear();
//  //    myDisplay.print(totalError);
//  //    // Update Motor PWM values
//  setLeftPWM(currentLeftPWM);
//  setRightPWM(currentRightPWM);
//
//  if (i == 50) {
//    //    myDisplay.clear();
//  }
//  if (continueTurn) {
//    if (!straight) {
//      i++;
//    }
//  }
//  else {
//    i = 0;
//    //    if (moveType == TURN_RIGHT) {
//    //      angle -= 90;
//    //      encoderAngle -= 90;
//    //    }
//    //    else {
//    //      angle += 90;
//    //      encoderAngle += 90;
//    //    }
//    angle = 0;
//    encoderAngle = 0;
//    //    prevRightTicks -= rightTicks;
//    //    prevLeftTicks -= leftTicks;
//    gyroSensitivity = .0036;
//    prevRightTicks -= rightTicks;
//    prevLeftTicks -= leftTicks;
//    rightTicks = 0;
//    leftTicks = 0;
//    moveType = NO;
//    walls_global[0] = wallLeft();
//    walls_global[1] = wallFront();
//    walls_global[2] = wallRight();
//    currentMoveDone = true;
//    //needMove = true;
//    //straight = false;
//    turn = false;
//    straight = false;
//    continueTurn = true;
//    wallInFront = false;
//  }
//}

void curveTurn() {
  double errorP;
  static double oldErrorP;
  double errorD;
  double totalError;
  double Kp = 10;//TODO May need to scale with speed ex: goalspeed/4
  //double Kd = 10;
  double targetAngle;
  //long long start = millis();
  //int timeConst = 2 * turnSpeed; //ms 2
  static bool continueTurn = true;
  static bool turn = false;
  static bool straight = false;
  static bool wallInFront = false;
  int radius = 65; //mm
  static float angularAccel; //deg/s/s
  static float angularVel; //deg/s
  static int t1; //ms
  static int t2; //ms
  static int t3; //ms
  static int totalTimeIdeal; //ms
  static int totalTimeActual; //ms
  static float degS = 0; //deg/s
  static float degSPrev = 0; //deg/s
  static float degrees = 0; //deg
  static float degreesPrev = 0; //deg
  static int time = 0; //ms
  const int tickStop = 1900;

  if (time == 0) {
    angle = 90.0;
    // Do initial calculations at the beginning
    angularAccel = ((7.4755 * pow(angle, -.383)) * goalSpeed);
    angularAccel *= angularAccel / radius;
    angularVel = sqrt(angularAccel / radius) * 180 / M_PI;
    totalTimeIdeal = angle * M_PI * radius / 180 / goalSpeed * 1000 + 0.5;
    t1 = angularVel / angularAccel * 1000 + 0.5;
    t2 = (angle / angularVel * 1000 + 0.5) - t1;
    t3 = t1;
    totalTimeActual = t1 + t2 + t3;
    degSPrev = 0;
    degreesPrev = 0;
    encoderAngle = 0;
  }

  if (turn == false) {
    if (wallFront()) {
      wallInFront = true;
      targetAngle = 0;// TODO do forward correction during this part?
      if ((rightFront + leftFront) / 2 > frontStop - 0) {
        turn = true;
        time = 1;
        //        i = round(pow(11.346 * abs(angleConst*(rightFront - leftFront)),0.5492));//how does R-L relate to angle?
        //                angle = 1*(rightFrontRaw - leftFrontRaw);
        //        angle = 0.0;
      }
    }
    else {
      //      myDisplay.setCursor(0);
      //      myDisplay.clear();
      //      myDisplay.print("NoWF");
      if ((leftTicks + rightTicks) / 2 >= tickStop) {//used to be 1500 ticks
        //        if ((moveType == TURN_RIGHT && !wallRight()) || moveType == TURN_LEFT && !wallLeft()) {
        turn = true;
        time = 1;
      }
    }
    //turn = true;
  }
  else {
    if (time <= totalTimeActual) {
      if (time <= t1) {
        degS = angularAccel / 1000 + degSPrev;
      }
      else {
        if (time <= (t1 + t2))
        {
          degS = angularVel;
        }
        else {
          degS = degSPrev - angularAccel / 1000;
        }
      }
    }
    else {
      degS = 0;
    }
    degrees = degS / 1000 + degreesPrev;
    degreesPrev = degrees;
    degSPrev = degS;
    if (moveType == TURN_RIGHT) {
      targetAngle = degrees;
    }
    else {
      targetAngle = -degrees;
    }
  }

  if (moveType == TURN_RIGHT) {
    if (straight == false && time > totalTimeActual - 1) {
//      encoderAngle = 90;
      targetAngle = 90;
    }
  }
  if (moveType == TURN_LEFT) {
    if (straight == false && time > totalTimeActual - 1) {
//      encoderAngle = -90;
      targetAngle = -90;
    }
  }

  if (straight == false && abs(targetAngle) == 90) {
    prevRightTicks -= rightTicks;
    prevLeftTicks -= leftTicks;
    rightTicks = 0;
    leftTicks = 0;
    straight = true;
  }
  if (straight == true) {
    if (moveType == TURN_RIGHT) {
      targetAngle = 90;
    }
    else {
      targetAngle = -90;
    }
    if (wallFront()) {
      if ((leftFront + rightFront) / 2 > frontStop) {
        continueTurn = false;
      }
    }
    else {
      if ((rightTicks + leftTicks) / 2 > tickStop) {
        continueTurn = false;
      }
    }
//    if ((rightTicks + leftTicks) / 2 > 1500  || (leftFront + rightFront) / 2 > frontStop) {//used to be 1500 ticks
//      continueTurn = false;
//    }
  }

  if ((targetAngle == 0 || abs(targetAngle) == 90) && wallFront()) {
    errorP = .02 * (rightFrontRaw - leftFrontRaw) + targetAngle - encoderAngle;
    //    errorP = 2 * (rightFront - leftFront);
    //        errorP = targetAngle - angle;
    //        errorP = .5 * (rightFront - leftFront - 0.0) + targetAngle - angle;
  }
  else {
    errorP = targetAngle - encoderAngle;
  }
  errorD = errorP - oldErrorP;
  totalError = 40 * errorP + 10 * errorD;
  // Calculate PWM based on Error
  currentLeftPWM = leftBaseSpeed + int(totalError);
  currentRightPWM = rightBaseSpeed - int(totalError);
  //    myDisplay.setCursor(0);
  //    myDisplay.clear();
  //    myDisplay.print(totalError);
  //    // Update Motor PWM values
  setLeftPWM(currentLeftPWM);
  setRightPWM(currentRightPWM);

  if (time == 50) {
    myDisplay.clear();
  }
  if (continueTurn) {
    if (!straight) {
      time++;
    }
  }
  else {
    myDisplay.clear();
    myDisplay.setCursor(0);
    myDisplay.print("stop");
    time = 0;
    angle = 0.0;
    encoderAngle = 0;
    prevRightTicks -= rightTicks;
    prevLeftTicks -= leftTicks;
    rightTicks = 0;
    leftTicks = 0;
    moveType = NO;
    walls_global[0] = wallLeft();
    walls_global[1] = wallFront();
    walls_global[2] = wallRight();
    currentMoveDone = true;
    turn = false;
    straight = false;
    continueTurn = true;
    wallInFront = false;
  }
}

void pivotTurnRight90() {
  int errorP;
  int errorD;
  int totalError;
  int oldErrorP;
  int tickCount = 190;
  int finishedCount = 0;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 250;
  const float targetDegrees = 90;
  float targetAngle = 0;
  // const int turnSpeed = 45;
  // const int targetDegrees = 85.5
  // const int turnSpeed = 40;
  // const int targetDegrees = 86
  float initialZ;
  long count = 0;
  int i = 0;
  gyroSensitivity = 0.0036;
  //    rightTicks = 0;
  //    leftTicks = 0;
  //    delay(200);

  //    while (rightTicks > -90 || leftTicks < 90) {
  //    }
  leftTicks = 0;
  rightTicks = 0;
  prevLeftTicks = 0;
  prevRightTicks = 0;
  angle = 0;
  encoderAngle = 0;
  degreesTraveled = 0;
  count = micros();
  while (targetAngle <= targetDegrees) {
    readGyro();
    getSpeed();
//    Serial3.print(leftSpeed); Serial3.print(",");Serial3.println(rightSpeed);
    readEncoderAngle();
    delay(1);
//    myDisplay.setCursor(0);
//    myDisplay.clear();
//    myDisplay.print(encoderAngle);
    targetAngle = encoderAngle;
    setLeftPWM(turnSpeed);
    setRightPWM(-turnSpeed);
    /*
      uint32_t deltat = micros() - count;
      if (deltat > 1000) {
      getSpeed();
      if (targetAngle < 90) {
        targetAngle = curve4[i];
        i++;
      }
      else {
        finishedCount++;
      }
      readGyro();
      errorP = angle - targetAngle;// + .1 * (leftSpeed + rightSpeed);
      errorD = errorP - oldErrorP;
      totalError = 39 * errorP + 20 * errorD;
      currentLeftPWM = -totalError;
      currentRightPWM = totalError;
      setLeftPWM(currentLeftPWM);// + (int)(.5 * (abs(rightTicks) - leftTicks)));
      setRightPWM(currentRightPWM);
      oldErrorP = errorP;
      count = micros();
      if (finishedCount > 5) {
        //        while (currentLeftPWM > 0 && currentRightPWM > 0) {
        //          if (currentLeftPWM > 0) {
        //            setLeftPWM(currentLeftPWM--);
        //          }
        //          if (currentRightPWM < 0) {
        //            setRightPWM(currentRightPWM++);
        //          }
        //        }
        //delay(100);
        break;
      }
      }
    */
  }
  /*
    while (currentLeftPWM > 0 && currentRightPWM < 0) {
    if (currentLeftPWM > 0) {
      currentLeftPWM--;
      setLeftPWM(currentLeftPWM);
    }
    if (currentRightPWM < 0) {
      currentRightPWM++;
      setRightPWM(currentRightPWM);
    }
    delay(1);
    }
  */
  setLeftPWM(0);
  setRightPWM(0);
  //  prevLeftTicks -= leftTicks;
  //  prevRightTicks -= rightTicks;
  leftTicks = 0;
  rightTicks = 0;
  prevLeftTicks = 0;
  delay(100);
  //    // Needs to deccelerate for the motors to stop correctly
  //  for (int i = turnSpeed; i >= 0; --i) {
  //    setLeftPWM(i);
  //    setRightPWM(-i);
  //  }
}

// If mouse starts facing a wall, this turns in the first cell to face the opening
void rightTurnFirstCell() {
  const int frontLeftStop = 1900;
  const int frontRightStop = 1900;
  bool leftStop = false;
  bool rightStop = false;
  int tickCount = 180;
  int errorP;
  int errorD;
  int totalError;
  bool front;
  //  leftBaseSpeed = exploreSpeed;
  //  rightBaseSpeed = exploreSpeed;
  moveType = NO;

  //Turn Around with no wall in front

  const int tickValue = 50;
  while (leftFront < frontLeftStop && rightFront < frontRightStop) {
    // Only left wall
    // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
    //read Gyro? TODO
    //errorP = 20 * (angle) + .5 * (leftSensor - leftWallDist);
    errorD = errorP;

    errorD = errorP;
    totalError = straightKp * errorP + Kd * errorD;

    // Calculate PWM based on Error
    currentLeftPWM = leftBaseSpeed + totalError / 124;
    currentRightPWM = rightBaseSpeed - totalError / 124;

    // Update Motor PWM values
    setLeftPWM(currentLeftPWM);
    setRightPWM(currentRightPWM);

    //TODO (this is a hack and shouldn't be here, but it makes it work)
    haveSensorReading = false;
    while (!haveSensorReading) {
      readSensors();
      delayMicroseconds(80);
    }

  }
  setRightPWM(0);
  setLeftPWM(0);
  delay(200);

  pivotTurnRight90();

  angle = 0.0;
  encoderAngle = 0;
  delay(200);
  setLeftPWM(-150);
  setRightPWM(-150);
  delay(350);
  for (int i = -150; i < 0; ++i) {
    setLeftPWM(i);
    setRightPWM(i);
  }
  delay(200);
  firstCell = true;
  movesBuffer[0] = 'f';

}
