#include "kyleEEPROM.h"

void writeEEPROM(bool write) {
  if (write) {
    for (int i = 0; i < Maze::WIDTH*Maze::HEIGHT; i++) {
      //EEPROM.update(address, data);
      EEPROM.update(i, Maze::m_data[i]);
    }
    shouldWriteEEPROM = false;
  }
}

void readEEPROM() {
  for (int i = 0; i < Maze::WIDTH*Maze::HEIGHT; i++) {
    if (recoverMaze) {
      Maze::m_data[i] = EEPROM.read(i);
    }
    else {
      Maze::m_data[i] = 0;
    }
  }
}
