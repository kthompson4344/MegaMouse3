void chooseMode() {
  int tickValue = 200;
  myDisplay.clear();
  myDisplay.setCursor(0);
  myDisplay.print("Load");
  delay(750);
  myDisplay.clear();
  myDisplay.setCursor(0);
  myDisplay.print("Maze");
  delay(750);
  while(1) {
    if (leftTicks > tickValue * 2) {
      leftTicks = 0;
    }
    if (leftTicks > tickValue) {
      recoverMaze = true;
    }
    else {
      recoverMaze = false;
    }
    myDisplay.clear();
    myDisplay.setCursor(0);
    if (recoverMaze) {
      myDisplay.print("Yes");
    }
    else {
      myDisplay.print("No");
    }
    if (digitalRead(buttonPin) == 0) {
      break;
    }
  }
}

void chooseSpeed() {
  leftTicks = exploreSpeed * 10;
  myDisplay.clear();
  myDisplay.setCursor(0);
  myDisplay.print("mm/s");
  delay(1000);
  while(1) {
    myDisplay.clear();
    myDisplay.setCursor(0);
    myDisplay.print(int(leftTicks / 10));
    delay(10);
    if (digitalRead(buttonPin) == 0) {
      exploreSpeed = leftTicks / 10;
      leftTicks = 0;
      break;
    }
  }
}

void chooseAcceleration() {
  int tickValue = 200;
  myDisplay.clear();
  myDisplay.setCursor(0);
  myDisplay.print("Acce");
  delay(400);
  myDisplay.clear();
  myDisplay.setCursor(0);
  myDisplay.print("lera");
  delay(400);
  myDisplay.clear();
  myDisplay.setCursor(0);
  myDisplay.print("tion");
  delay(400);
  leftTicks = 0;
  while(1) {
    if (leftTicks > tickValue * 2) {
      leftTicks = 0;
    }
    if (leftTicks > tickValue) {
      accelerate = true;
    }
    else {
      accelerate = false;
    }
    myDisplay.clear();
    myDisplay.home();
    if (accelerate) {
      myDisplay.print("Yes ");
    }
    else {
      myDisplay.print("No  ");
    }
    if (digitalRead(buttonPin) == 0) {
      break;
    }
  }
  leftTicks = 0;
}

