// Teensy 3 I2C Library
#include <LedDisplay.h>
#include "Motors.h"
#include "Sensors.h"
#include "Profiles.h"
#include "Algo.h"
#include <EEPROM.h>
Algo algo;
Interface interface;

/*********************************************************
  //MAZE SIZE MUST BE CHANGED IN INTERFACE.CPP AND MAZE.H
*********************************************************/
#define dataPin 11              // connects to the display's data in
#define registerSelect 12       // the display's register select pin 
#define clockPin 13             // the display's clock pin
#define enable 10               // the display's chip enable pin
#define reset 9                 // the display's reset pin
#define displayLength 4         // number of characters in the display

// create an instance of the LED display library:
LedDisplay myDisplay = LedDisplay(dataPin, registerSelect, clockPin, enable, reset, displayLength);
int brightness = 15;        // screen brightness

//Thresholds for left and right sensors detecting side walls
#define hasLeftWall 450
#define hasRightWall 450

//Seperate speeds for explore and solve (mm/s) (not currently implemented)
int exploreSpeed = 400;
int solveSpeed = 1000;

float leftBaseSpeed = exploreSpeed;
float rightBaseSpeed = exploreSpeed;

float leftSpeed;
float rightSpeed;

const float frontStop = 6.6;//3.7
//float gyroZeroVoltage = 1.55;
// PID Constants
double straightKp;
#define Kd 0
double errorP;

/* Variables for interface between drive code and algorithm */
volatile bool buttonPressed;
volatile char movesBuffer[256];
byte moveBufferIndex;
char bluetoothBuffer[5];
volatile bool walls_global[3] = {false, false, false}; // Left, Front, Right
volatile bool movesReady = false; // Set to true by algorithm, set to false by drive.
volatile bool movesDoneAndWallsSet = false; // Set to true by drive, set to false by algorithm.
volatile bool shouldWriteEEPROM = false; //Set to true by algorithm, set to false by drive.
/* End of variables for interface */

//Max speed for acceleration
const int maxSpeed = 1000;

const int maxPWM = 2000;

bool currentMoveDone = false;
bool firstMove = true;
bool accelerate = false;
bool solving = 0;
int goalSpeed = 0;
float gyroZeroVoltage;
float gyroSensitivity = .0037;

volatile bool firstCell = true;
volatile bool afterTurnAround = false;

//Walls currently on left or right
volatile bool rightValid = true;
volatile bool leftValid = true;

volatile bool haveSensorReading = false;

IntervalTimer correctionTimer;
IntervalTimer sensorTimer;
IntervalTimer refreshSensorTimer;
//SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

//Current angle of the robot
volatile float angle = 0.0;
volatile float encoderAngle = 0.0;
volatile float gyroAngle = 0.0;
volatile float offsetAngle = 0;

//Different move types
volatile enum {
  NO = 0,
  FORWARD = 1,
  TURN_LEFT = 2,
  TURN_RIGHT = 3,
  TURN_AROUND = 4
} moveType;

const int buttonPin = 30;

bool recoverMaze = false;

int sensorCounts = 0;

void setup() {
  //Serial.begin(9600);
  //  Serial.begin(115200);
  Serial3.begin(115200);
  myDisplay.begin();
  // set the brightness of the display:
  myDisplay.setBrightness(brightness);
  myDisplay.setCursor(0);
  myDisplay.println("Mega");
  // 12 bit ADC resolution
  analogReadResolution(12);

  setupMotors();
  setupSensors();

  pinMode(buttonPin, INPUT_PULLUP);

  chooseMode();
  chooseSpeed();
  chooseAcceleration();
  if (exploreSpeed > 750) {
      straightKp = 3;
    }
    else if (exploreSpeed > 590) {
      straightKp = 2;
    }
    else {
      straightKp = 1.5;
    }
  moveType = NO;
  delay(1000);
  myDisplay.setCursor(0);
  myDisplay.clear();
  myDisplay.print("3");
  delay(1000);
  myDisplay.clear();
  myDisplay.setCursor(1);
  myDisplay.print("2");
  delay(1000);
  myDisplay.clear();
  myDisplay.setCursor(2);
  myDisplay.print("1");
  delay(1000);
  myDisplay.clear();
  setupGyro();
  //Reads a different sensor every 80us
  sensorTimer.priority(250);
  sensorTimer.begin(readSensors, 80);
  while (!haveSensorReading) {
  }

  //read initial sensor values to determine first cell move
  walls_global[0] = wallLeft();
  walls_global[1] = wallFront();
  walls_global[2] = wallRight();

  haveSensorReading = false;
  movesDoneAndWallsSet = true;
  
  attachInterrupt(buttonPin, resetPressed, FALLING);
  
  //Runs every 1ms and controls mouse movements
  correctionTimer.priority(255);
  correctionTimer.begin(correction, 1000);
  //  setLeftPWM(0);
  //  setRightPWM(0);
  rightTicks = 0;
  leftTicks = 0;
}

void loop() {
  //Solve the maze
//    if (analogRead(A14) /4096.0 * 3.3 / 0.357 < 6.5) {
//      while(1) {
//        setLeftPWM(0);
//        setRightPWM(0);
//        myDisplay.clear();
//        myDisplay.setCursor(0);
//        myDisplay.print("bat");
//        delay(200);
//        myDisplay.clear();
//        myDisplay.setCursor(0);
//        myDisplay.print("");
//        delay(200);
//      }
//    }
//  solve();
  algo.solve(&interface);
}

void getSpeed() {
  const int timeConst = 1;//ms
  //  static int count = 0;
  // 0.55 mm/tick (16 count)
  //  const float mmPT = 0.55;
  // 0.017 mm/tick (512 count)
  const float mmPT = 0.017349;
  //    if (count >= timeConst) {
  leftSpeed = (leftTicks - prevLeftTicks) * mmPT / (timeConst / 1000.0);
  rightSpeed = (rightTicks - prevRightTicks) * mmPT / (timeConst / 1000.0);
  prevLeftTicks = leftTicks;
  prevRightTicks = rightTicks;
}

void setSpeed() {
  float avgSpeed = (leftSpeed + rightSpeed) / 2.0;
  if ((leftTicks == 0 || rightTicks == 0)) {
    if (firstCell || afterTurnAround) {
      leftSpeed = 0;
      rightSpeed = 0;
    }
    else {
      leftSpeed = goalSpeed;
      rightSpeed = goalSpeed;
    }
  }
  //      float avgSpeed = (leftSpeed + rightSpeed) / 2.0;
  if (moveType == FORWARD) {
    if (leftSpeed >= 0 && rightSpeed >= 0) {
      if (goalSpeed > 0) {
        if (avgSpeed < goalSpeed) {
          if (accelerate && !afterTurnAround && !firstCell) {//TODO
            leftBaseSpeed += .3;
            rightBaseSpeed += .3;
          }
          else {
            leftBaseSpeed += (.0015*goalSpeed - .4);
            rightBaseSpeed += (.0015*goalSpeed - .4);
          }
        } else if (avgSpeed > goalSpeed) {
          if (accelerate && !afterTurnAround && !firstCell) {//TODO
            leftBaseSpeed -= .3;
            rightBaseSpeed -= .3;
          }
          else {
            leftBaseSpeed -= (.0015*goalSpeed - .4);
            rightBaseSpeed -= (.0015*goalSpeed - .4);
          }
        } else {
          //      count = 0;
        }
      }
    }
    //      myDisplay.clear();
    //      myDisplay.setCursor(0);
    //      myDisplay.print(rightBaseSpeed);
    //      count = 0;
    //    }
    //    if (leftBaseSpeed < 105 && goalSpeed > 0) {
    //
    //      leftBaseSpeed += .1;
    //      rightBaseSpeed += .1;
    //    }
    //        if (!firstCell && !afterTurnAround && accelerate && goalSpeed - avgSpeed > 50) {
    //          leftBaseSpeed++;
    //          rightBaseSpeed++;
    //        }
    if (goalSpeed == 0 && leftBaseSpeed > 0) {
      leftBaseSpeed -= .2;
      rightBaseSpeed -= .2;
    }
    //    if (avgSpeed - goalSpeed > 50) {
    //      leftBaseSpeed -= .05;
    //      rightBaseSpeed -= .05;
    //    }
    if (goalSpeed == 0) {
      if (leftBaseSpeed < 0) {
        leftBaseSpeed = 0;
      }
      if (rightBaseSpeed < 0) {
        rightBaseSpeed = 0;
      }
    }

  }//END IF FORWARD
  //  leftBaseSpeed = 240;
  //  rightBaseSpeed = 240;
  //TODO: Decelleartion
  //  count++;
  //  }
}

//1ms timer
void correction() {
  static int totalForwardCount = 0;
  static int forwardCount = 0;
  static bool in_acceleration = false;
  static byte indexInBuffer = 0;
  static bool movedForward = false;
  static int reportingCount = 0;
  getSpeed();
  setSpeed();
  readGyro();
  readEncoderAngle();
  reportingCount++;
  if (reportingCount >= 10) {
    reportingCount = 0;
    //    Serial3.print(angle);Serial3.print(",");Serial3.println(encoderAngle);
  }
  //  angle = (angle + encoderAngle) / 2.0;
  if (!movesReady) {
    // Hoping we never get here, but maybe the algorithm is slow.
    haveSensorReading = false;
    return;
  }

  if (currentMoveDone) {
    movedForward = false;
    if (firstMove) {
      firstMove = false;
    }
    if (forwardCount != 0) {
      forwardCount--;
    }
    else {
      totalForwardCount = 0;
      in_acceleration = false;
      accelerate = false;
    }
    indexInBuffer += 1;
    currentMoveDone = false;
    if (movesBuffer[indexInBuffer] == 0) {
      // Make sure walls_global is set by the time we get here (STILL NEED TO DO IN TURN AROUND).
      movesReady = false;

      //mySerial.print(walls_global[0]);
      //mySerial.print(walls_global[1]);
      //mySerial.println(walls_global[2]);
      movesDoneAndWallsSet = true;
      indexInBuffer = 0;
      haveSensorReading = false;
      return;
    }
  }

  switch (movesBuffer[indexInBuffer]) {
    case 'f':
      if (!in_acceleration) {
        forwardCount = 1;
        int index = indexInBuffer + 1;
        while (movesBuffer[index++] == 'f') {
          forwardCount++;
        }
        totalForwardCount = forwardCount;
        if (solving) {
          goalSpeed = (int)solveSpeed * (float)forwardCount;
          if (goalSpeed > maxSpeed)
            goalSpeed = maxSpeed;
          if (goalSpeed < solveSpeed)
            goalSpeed = solveSpeed;
        }
        else {
          if (accelerate) {
            goalSpeed = (int)exploreSpeed * (int)forwardCount;
            if (goalSpeed < exploreSpeed)
              goalSpeed = exploreSpeed;
            if (goalSpeed > maxSpeed)
              goalSpeed = maxSpeed;
          }
          else {
            goalSpeed = exploreSpeed;
          }
        }
        in_acceleration = true;
//        accelerate = true;
      }
      else {
        // if((float)totalForwardCount / (float)(forwardCount) == totalForwardCount) {
        //        myDisplay.setCursor(0);
        //        myDisplay.clear();
        //        myDisplay.print(goalSpeed);
        if (totalForwardCount > 10 && forwardCount == 4) {//TODO 2
          goalSpeed = exploreSpeed;//TODO
        }
        else if (totalForwardCount >= 4 && forwardCount == 3) {
          goalSpeed = exploreSpeed;
        }
        else if (totalForwardCount >= 1 && forwardCount == 2) {
          goalSpeed = exploreSpeed;
        }
      }
      moveType = FORWARD;

      if (!movedForward) {
        movedForward = true;
        moveForward();
      }
      forwardCorrection();
      break;
    case 'r':
      moveType = TURN_RIGHT;
      if (firstMove) {
        correctionTimer.end();
        sensorTimer.end();
        leftTicks = 0;
        rightTicks = 0;
        rightTurnFirstCell();
        correctionTimer.priority(255);
        walls_global[0] = wallLeft();
        walls_global[1] = wallFront();
        walls_global[2] = wallRight();
        haveSensorReading = false;
        correctionTimer.begin(correction, 1000);
        sensorTimer.begin(readSensors, 80);
        return;
      }
      curveTurn();
      //      curveTurn();
      break;
    case 'l':
      moveType = TURN_LEFT;
      curveTurn();
      //      curveTurn();
      break;
    case 'a':
      correctionTimer.end();
      sensorTimer.end();
      leftTicks = 0;
      rightTicks = 0;
      moveType = TURN_AROUND;
      turnAround();
      currentMoveDone = true;
      haveSensorReading = false;
      correctionTimer.priority(255);
      correctionTimer.begin(correction, 1000);
      sensorTimer.begin(readSensors, 80);
      return;
    default:
      moveType = NO;

      // Don't need to do anything here if we're turning around.
  }

  haveSensorReading = false;
}

void moveForward() {
  //    myDisplay.setCursor(0);
  //    myDisplay.clear();
  //    myDisplay.print("Fd");
  if (firstCell) {
    rightTicks = 3500;//70
    leftTicks = 3500; //70
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;
    //    accelerate = true;
  }
  else if (afterTurnAround) {
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;
    //    accelerate = true;
    rightTicks = 5640;//140
    leftTicks = 5640;//140
    prevRightTicks = 0;
    prevLeftTicks = 0;
  }
  else {
    rightTicks = 0;
    leftTicks = 0;
  }

  rightValid = wallRight();
  leftValid = wallLeft();
  myDisplay.clear();
  myDisplay.setCursor(0);
  myDisplay.print(leftValid); myDisplay.print(" "); myDisplay.print(rightValid);
  moveType = FORWARD;
}

void turnAround() {
  bool stop = false;
  int tickCount = 180 * 32;
  int errorD;
  int totalError;
  bool front;
  int waitTime = 50;
  //  gyroZeroVoltage = 1.56;
  //  leftBaseSpeed = 200;
  //  rightBaseSpeed = 200;
  myDisplay.clear();
  myDisplay.setCursor(0);
  myDisplay.print("arnd");
  if ((rightFrontRaw + leftFrontRaw) / 2 > 400) {
    front = true;
    afterTurnAround = true;
  }
  else {
    front = false;
    afterTurnAround = true;
  }
  if (front) {
    while (1) {
      //      refreshSensor();
      getSpeed();
      setSpeed();
      readGyro();
      const float frontValue = frontStop;
      if ((leftFront + rightFront) / 2 > frontValue && stop == false) {
        stop = true;
        goalSpeed = 0;//TODO DO THE SAME FOR NO WALL
      }
      if (stop == true) {
        if (rightBaseSpeed > 0) {
          rightBaseSpeed-=(.01*exploreSpeed - 3);
          leftBaseSpeed-=(.01*exploreSpeed - 3);
        }
        else {
          leftBaseSpeed = 0;
          rightBaseSpeed = 0;
          setLeftPWM(0);
          setRightPWM(0);
          delay(waitTime);
          break;
        }
      }
      straightCorrection(wallLeft(), wallRight(), wallFront());

      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
      delayMicroseconds(500);
    }

  }

  //Turn Around with no wall in front
  else {
    while (1) {
      //      refreshSensor();
      getSpeed();
      setSpeed();
      readGyro();
      const int tickValue = 10375-5700;
      if ((leftTicks + rightTicks) / 2 > tickValue && stop == false) {

        stop = true;
      }
      if (stop == true) {
        if (rightBaseSpeed > 30) {
          rightBaseSpeed-=(.01*exploreSpeed - 3);
          leftBaseSpeed-=(.01*exploreSpeed -3);
        }
        else {
          leftBaseSpeed = 0;
          rightBaseSpeed = 0;
          setLeftPWM(0);
          setRightPWM(0);
          delay(waitTime);
          break;
        }
      }

      straightCorrection(wallLeft(), wallRight(), 0);

      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
    }
  }
  rightTicks = 0;
  leftTicks = 0;
  //  delay(5000);
  stop = false;
  leftBaseSpeed = 200;//TODO
  rightBaseSpeed = 200;
  moveType = NO;
  //  myDisplay.clear();
  //  myDisplay.setCursor(0);
  int i = 0;
  delayMicroseconds(80);
  // Read right front sensor, then turn it off.
  haveSensorReading = false;
  while (!haveSensorReading) {
    readSensors();
    delayMicroseconds(80);
  }
  if ((rightFrontRaw + leftFrontRaw) / 2 > 400) {
    while (i < waitTime) {
      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
      if (!wallFront()) {
        break;
      }
      else {
        if ((leftFrontRaw + rightFrontRaw) / 2 > 2500) {
          setLeftPWM(int(2 * (3800 - leftFrontRaw)));
          setRightPWM(int(2 * (3770 - rightFrontRaw)));
        }
        else if ((leftFrontRaw + rightFrontRaw) / 2 > 400) {
          setLeftPWM(int(1 * (3800 - leftFrontRaw)));
          setRightPWM(int(1 * (3770 - rightFrontRaw)));
        }
        else {
          setLeftPWM(int(.5 * (3710 - leftFrontRaw)));
          setRightPWM(int(.5 * (3680 - rightFrontRaw)));
        }
      }
      i++;
      delay(1);
    }
    setLeftPWM(0);
    setRightPWM(0);
    i = 0;
    while (i < waitTime) {
      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
      setLeftPWM(int(10 * (rightFrontRaw - leftFrontRaw)));
      setRightPWM(int(10 * (leftFrontRaw - rightFrontRaw)));
      i++;
      delay(1);
    }
    setLeftPWM(0);
    setRightPWM(0);
  }
  writeEEPROM(shouldWriteEEPROM);
  pivotTurnRight90();
  //  myDisplay.print("Done");

  //use only for pivotTurn90
  i = 0;
  haveSensorReading = false;
  while (!haveSensorReading) {
    readSensors();
    delayMicroseconds(80);
  }
  if (wallFront()) {
    while (i < waitTime + 100) {
      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
      if ((leftFrontRaw + rightFrontRaw) / 2 < 100) {
        break;
      }
      else {
        if ((leftFrontRaw + rightFrontRaw) / 2 > 2500) {
          setLeftPWM(int(1 * (3800 - leftFrontRaw)));
          setRightPWM(int(1 * (3770 - rightFrontRaw)));
        }
        else if ((leftFrontRaw + rightFrontRaw) / 2 > 1000) {
          setLeftPWM(int(.5 * (3800 - leftFrontRaw)));
          setRightPWM(int(.5 * (3770 - rightFrontRaw)));
        }
        else {
          setLeftPWM(int(.2 * (3800 - leftFrontRaw)));
          setRightPWM(int(.2 * (3770 - rightFrontRaw)));
        }
      }
      i++;
      delay(1);
    }
    setLeftPWM(0);
    setRightPWM(0);
    i = 0;
    while (i < waitTime) {
      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
      setLeftPWM(int(10 * (rightFrontRaw - leftFrontRaw)));
      setRightPWM(int(10 * (leftFrontRaw - rightFrontRaw)));
      i++;
      delay(1);
    }
  }
  setLeftPWM(0);
  setRightPWM(0);
  pivotTurnRight90();
  
  prevRightTicks = 0;
  prevLeftTicks = 0;
  leftTicks = 0;
  rightTicks = 0;
  angle = 0.0;
  encoderAngle = 0;
  offsetAngle = 0;
}

void straightCorrection(bool left, bool right, bool front) {
  int errorD;
  static int oldErrorP = 0;
  int totalError;
  if (front) {
    errorP = 1 * (rightFrontRaw - leftFrontRaw);
  }
  else if (right && left) {
    errorP = 10 * (leftMiddleValue - rightMiddleValue);
//    angle = 0;
        encoderAngle = 0;
  }
  else if (right) {
    // Only right wall
    errorP = 10 * (1000 - rightMiddleValue) - 20 * encoderAngle;
  }
  else if (left) {
    // Only left wall
    // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);

    errorP = 10 * (leftMiddleValue - 1000) - 20 * encoderAngle;
  }
  else {
    //read Gyro? TODO
    //    errorP = 20 * (angle) + 1 * (rightTicks - leftTicks);
    //    errorP = 10 * (rightTicks - leftTicks);
    errorP = -50 * encoderAngle;
  }
  if (moveType == FORWARD || moveType == TURN_AROUND) {
    //      errorP += 3*(rightFront - leftFront);
    //  errorP = 0;
    errorD = oldErrorP;
    totalError = straightKp * errorP + Kd * errorD;
    oldErrorP = errorP;

    // Calculate PWM based on Error
    currentLeftPWM = leftBaseSpeed + (totalError / 124);
    currentRightPWM = rightBaseSpeed - (totalError / 124);
    if (currentLeftPWM < 0) {
      currentLeftPWM = 0;
    }
    if (currentRightPWM < 0) {
      currentRightPWM = 0;
    }
    // Update Motor PWM values
    setLeftPWM(currentLeftPWM);
    setRightPWM(currentRightPWM);
  }
}

void forwardCorrection() {
  const int oneCellTicks = 10375;
  const int noWallRight = 500;
  const int noWallLeft =  500;

  //  const int pegWallBack = 800;
  //  const int pegNoWalls = 1000;
  //  const int pegWallFront = 1000;
  //
  //  const int wallBackTicks = 232;
  //  const int noWallTicks = 209;
  //  const int frontWallTicks = 204;

  // encoder tick value when we check walls a cell ahead
  const int readingTicks = 215;
  // encoder tick value when we switch to next cell's values
  const int newSideTicks = 260;

  static bool nextRightValid;
  static bool nextLeftValid;
  static bool nextCellDecided = false;
  int errorD;
  static int oldErrorP = 0;
  int totalError;
  static int lastTicksL;
  static int lastTicksR;
  static float straightAngle = 0.0;
  static bool endCell = false;
  static bool currentWallLeft = true;
  static bool currentWallRight = true;
  static bool ticksDecided = false;
  static int count = 0;
  static int prevCorrection = 4;
  int avgTicks = (leftTicks + rightTicks) / 2;
  //  if (accelerate) {

  //  if (leftBaseSpeed == 0) {
  //    leftBaseSpeed = 30;
  //    rightBaseSpeed = 30;
  //  }
  //  }

  //  if ((leftFront + rightFront) / 2 >= frontStop) {
  //    rightTicks = oneCellTicks;
  //    leftTicks = oneCellTicks;
  //    prevRightTicks -= oneCellTicks;
  //    prevLeftTicks -= oneCellTicks;
  //  }
  if (rightMiddleValue <  noWallRight) {//TODO
    rightValid = false;
  }
  if (leftMiddleValue < noWallLeft) {//TODO
    leftValid = false;
  }
  //3090 stop correcting losing a wall
  //3090 - 5710 posts
  //4910 start correcting with added wall
  if (avgTicks >= 3090 && avgTicks < 5710) {
    leftValid = false;
    rightValid = false;
  }
  if (avgTicks >= 5710 && !nextCellDecided) {
    rightValid = wallRight();
    leftValid = wallLeft();
    nextCellDecided = true;
    myDisplay.clear();
    myDisplay.setCursor(0);
    myDisplay.print(leftValid); myDisplay.print(" "); myDisplay.print(rightValid);
  }
  /*
    // Next Cell Wall Detection
    if ((rightTicks + leftTicks) / 2 >= readingTicks && !nextCellDecided) {
    angle = 0;
    nextRightValid = rightMiddleValue > noWallRight;
    nextLeftValid = leftMiddleValue > noWallLeft;
    nextCellDecided = true;
    if (leftValid != nextLeftValid) {
      leftValid = false;
    }
    if (rightValid != nextRightValid) {
      rightValid = false;
    }
    }

    if ((rightTicks + leftTicks) / 2 >= newSideTicks) {
    leftValid = nextLeftValid;
    rightValid = nextRightValid;
    }
  */
  //Serial3.print(leftMiddleValue); Serial3.print(","); Serial3.print(rightMiddleValue); Serial3.print(","); Serial3.print(leftTicks); Serial3.print(","); Serial3.println(rightTicks);
  straightCorrection(leftValid, rightValid, 0);
  //  straightCorrection(false, false);

  if (avgTicks >= oneCellTicks) {
    endCell = true;
  }
  if (endCell) {
    oldErrorP = 0;
    ticksDecided = false;
    walls_global[0] = wallLeft();
    walls_global[1] = wallFront();
    walls_global[2] = wallRight();
    currentMoveDone = true;
    //        needMove = true;
    nextCellDecided = false;
    moveType = NO;
    endCell = false;
    //    leftTicks = 0;
    //    rightTicks = 0;
    prevRightTicks -= rightTicks;
    prevLeftTicks -= leftTicks;
    leftTicks = 0;
    rightTicks = 0;
    prevCorrection = 4;
    afterTurnAround = false;
    firstCell = false;
    offsetAngle = encoderAngle;
  }
}

void solve() {
  while (!movesDoneAndWallsSet) {
  }
  bool walls[3];
  walls[0] = walls_global[0];
  walls[1] = walls_global[1];
  walls[2] = walls_global[2];
  movesDoneAndWallsSet = false;

  movesBuffer[0] = 'f';
  movesBuffer[1] = 'f';
//  movesBuffer[2] = 'f';
//  movesBuffer[3] = 'f';
  movesBuffer[2] = 'a';
  //  movesBuffer[5] = 'f';
  //  movesBuffer[6] = 'f';
  //  movesBuffer[7] = 'f';
  //  movesBuffer[8] = 'a';
  //  movesBuffer[9] = 'f';
  //      movesBuffer[10] = 'r';
  //      movesBuffer[11] = 'l';
  //      movesBuffer[12] = 'f';
  //      movesBuffer[13] = 'r';
  //      movesBuffer[14] = 'f';
  //  movesBuffer[15] = 'l';
  //    movesBuffer[16] = 'f';
  //    movesBuffer[17] = 'r';
  //    movesBuffer[18] = 'a';
  //    movesBuffer[19] = 'f';

  movesReady = true;
}

void rightWallFollow() {
  while (!movesDoneAndWallsSet) {
  }
  bool walls[3];
  walls[0] = walls_global[0];
  walls[1] = walls_global[1];
  walls[2] = walls_global[2];
  myDisplay.setCursor(0);
  myDisplay.print(walls[0]);
  myDisplay.setCursor(1);
  myDisplay.print(walls[1]);
  myDisplay.setCursor(2);
  myDisplay.print(walls[2]);
  movesDoneAndWallsSet = false;
  if (!walls[2]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("right");
    movesBuffer[0] = 'r';
    movesBuffer[1] = 0;
  }
  else if (!walls[1]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("frwd");
    movesBuffer[0] = 'f';
    movesBuffer[1] = 0;
  }
  else if (!walls[0]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("left");
    movesBuffer[0] = 'l';
    movesBuffer[1] = 0;
  }
  else {
    movesBuffer[0] = 'a';
    movesBuffer[1] = 'f';
    movesBuffer[2] = 0;
  }
  movesReady = true;
}

void leftWallFollow() {
  bool walls[3];
  walls[0] = walls_global[0];
  walls[1] = walls_global[1];
  walls[2] = walls_global[2];
  movesDoneAndWallsSet = false;
  if (!walls[0]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("left");
    movesBuffer[0] = 'l';
    movesBuffer[1] = 0;
  }
  else if (!walls[1]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("frwd");
    movesBuffer[0] = 'f';
    movesBuffer[1] = 0;
  }
  else if (!walls[2]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("rght");
    movesBuffer[0] = 'r';
    movesBuffer[1] = 0;
  }
  else {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("arnd");
    movesBuffer[0] = 'a';
    movesBuffer[1] = 'f';
    movesBuffer[2] = 0;
  }
}

void resetPressed() {
  if (!buttonPressed) {
    buttonPressed = true;
  }
}

