void writeEEPROM(byte* maze, int size) {
  for (int i = 0; i < size; i++) {
    //EEPROM.update(address, data);
    EEPROM.update(i, maze[i]);
  }
}

void readEEPROM(byte* maze, int size) {
  for (int i = 0; i < size; i++) {
    maze[i] = EEPROM.read(i);
  }
}

