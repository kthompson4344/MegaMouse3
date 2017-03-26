#pragma once

const int DIRL = 17;
const int DIRR = 23; //63,62, MLA, MRB
const int PWML = 5; //61, 64, MRA,MLB
const int PWMR = 6;

const int ER1 = 32;
const int ER2 = 25;
const int EL1 = 4;
const int EL2 = 3;

// Motor Pins
const int motorPins[] = {DIRL, DIRR, PWML, PWMR};

// Encoder Pins
const int encoderPins[] = {ER1, ER2, EL1, EL2};

int currentRightPWM = 0;
int currentLeftPWM = 0;

// Encoder Ticks
volatile int rightTicks = 0;
volatile int leftTicks = 0;

int prevRightTicks;
int prevLeftTicks;
