
void setupMotors() {
  // Define Motor Pins as Outputs
  analogWriteResolution(11);
  for (int i = 0; i < 4; i++) {
    analogWriteFrequency(motorPins[i], 23437);
    pinMode(motorPins[i], OUTPUT);
  }
//  leftWheel.setup();
//  rightWheel.setup();
//  leftWheel.start();
//  rightWheel.start();

  // Define Encoder Pins as Inputs
  for (int i = 0; i < 4; i++) {
    pinMode(encoderPins[i], INPUT);
  }
  
  // Encoder Interrupt Setups
  attachInterrupt(ER1, rightEncoder1, RISING);
  attachInterrupt(ER2, rightEncoder2, RISING);
  attachInterrupt(EL1, leftEncoder1, RISING);
  attachInterrupt(EL2, leftEncoder2, RISING);
}

void setLeftPWM(int value) {
  if (value > maxPWM) {
    value = maxSpeed;
  }
  if (value < -maxPWM) {
    value = -maxSpeed;
  }
  if (value >= 0) {
    digitalWriteFast(DIRL, LOW);
    analogWrite(PWML, value);
  }

  else if (value == 0) {
    digitalWriteFast(DIRL, LOW);
    digitalWrite(PWML, LOW);
  }
  else {
    digitalWriteFast(DIRL, HIGH);
    analogWrite(PWML, abs(value));
  }
}

void setRightPWM(int value) {

  if (value > maxPWM) {
    value = maxSpeed;
  }
  if (value < -maxPWM) {
    value = -maxSpeed;
  }
  if (value >= 0) {
    digitalWriteFast(DIRR, LOW);
    analogWrite(PWMR, value);
  }
  else if (value == 0) {
    digitalWriteFast(DIRR, LOW);
    digitalWriteFast(PWMR, LOW);
  }
  else {
    digitalWriteFast(DIRR, HIGH);
    analogWrite(PWMR, abs(value));
  }
}

// Right Encoder ISR
void rightEncoder1() {
  if (digitalReadFast(ER2) == LOW) {
    rightTicks++;
  }
  else {
    rightTicks--;
  }
}


void rightEncoder2() {
  if (digitalReadFast(ER1) == HIGH) {
    rightTicks++;
  }
  else {
    rightTicks--;
  }
}


// Left Encoder ISR
void leftEncoder1() {
  if (digitalReadFast(EL2) == HIGH) {
    leftTicks++;
  }
  else {
    leftTicks--;
  }
}

void leftEncoder2() {
  if (digitalReadFast(EL1) == LOW) {
    leftTicks++;
  }
  else {
    leftTicks--;
  }
}
