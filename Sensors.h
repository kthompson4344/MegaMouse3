#pragma once

#define diagl 0
#define lf 1
#define rf 2
#define diagr 3

// IR Pair Pins
const int TX[4] = {31, 24,  33,  26};
const int RX[4] = {A16, A7, A6, A17};

volatile int rightMiddleValue;
volatile int leftMiddleValue;
volatile float leftFront;
volatile float rightFront;
volatile int leftFrontRaw;
volatile int rightFrontRaw;
