void setupSensors() {
  // Define IR Pair TXs as Outputs
  for (int i = 0; i < 4; i++) {
    pinMode(TX[i], OUTPUT);
    digitalWriteFast(TX[i], LOW);
  }

  //analogReadAveraging(5);
}

// Runs on a timer every 80 microseconds.  Works like a state machine, goes through groups of sensors individually
// and then updates their values.
void readSensors() {
  float leftLog;
  static int leftFrontAmbient = 0;
  static int rightFrontAmbient = 0;
  static int leftMiddleAmbient = 0;
  static int rightMiddleAmbient = 0;
  static int state = 0;
  if (!haveSensorReading) {
    switch (state) {
      case 0: // Read ambient values.
        leftFrontAmbient = analogRead(RX[lf]);
        rightFrontAmbient = analogRead(RX[rf]);
        leftMiddleAmbient = analogRead(RX[diagl]);
        rightMiddleAmbient = analogRead(RX[diagr]);
        break;

      case 1: // Turn on left front sensor.
        digitalWriteFast(TX[lf], HIGH);
        break;

      case 2: // Read left front sensor, then turn it off.
        leftFrontRaw = analogRead(RX[lf]);// - leftFrontAmbient;
        leftLog = 1 / log(leftFrontRaw);
        leftFront = 1 / (48.59 * (leftLog * leftLog) - 11.978 * leftLog + .8614);
        digitalWriteFast(TX[lf], LOW);
        break;

      case 3: // Turn on right front sensor.
        digitalWriteFast(TX[rf], HIGH);
        break;

      case 4: // Read right front sensor, then turn it off.
        rightFrontRaw = analogRead(RX[rf]);// - rightFrontAmbient;
        rightFront = log(rightFrontRaw);
        digitalWriteFast(TX[rf], LOW);
        break;

      case 5: // Turn on diagonal sensors.
        digitalWriteFast(TX[diagl], HIGH);
        digitalWriteFast(TX[diagr], HIGH);
        break;

      case 6: // Read diagonal sensors, then turn them off.
        leftMiddleValue = analogRead(RX[diagl]) - leftMiddleAmbient;
        rightMiddleValue = analogRead(RX[diagr]) - rightMiddleAmbient;
        digitalWriteFast(TX[diagl], LOW);
        digitalWriteFast(TX[diagr], LOW);
        break;
    }
    ++state;

    if (state > 6) {
      state = 0;
      haveSensorReading = true;
    }
  }
}


void refreshSensor() {
  haveSensorReading = false;
  while (!haveSensorReading) {
  }
}

bool wallFront() {
  float value = 300;//13-15
  return ( (leftFrontRaw + rightFrontRaw) / 2.0 >= value);
  //  return (leftFrontRaw > value && rightFrontRaw > value);

}

bool wallLeft() {
  return (leftMiddleValue > 400);//200
  //  return (leftSensor > hasLeftWall);
}

bool wallRight() {
//  myDisplay.setCursor(0);
//  myDisplay.clear();
//  myDisplay.print(rightMiddleValue);
  return (rightMiddleValue > 400);
  //  return (rightSensor > hasRightWall);
}

void printSensors() {
  Serial.print(leftFrontRaw); Serial.print(" ");
  Serial.print(leftMiddleValue); Serial.print(" ");
  Serial.print(rightMiddleValue); Serial.print(" ");
  Serial.println(rightFrontRaw);
}

