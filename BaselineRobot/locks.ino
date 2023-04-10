// lock routines
int lockSet(int &lock, int value) {
  int retVal = 0;
  if (!value) return retVal;
  noInterrupts();
  if (lock != 0) {
    retVal = 1;
  } else {
    lock = value;
    retVal = 0;
  }
  interrupts();
  return retVal;
}

int lockUnSet(int &lock, int value) {
  int retVal = 0;
  if (!value) return retVal;
  noInterrupts();
  if (!lock) {
    retVal = 1;
  } else if (lock != value) {
    retVal = 2;
  } else {
    lock = 0;
    retVal = 0;
  }
  interrupts();
  return retVal;
}

int lockTest(int &lock, int value) {
  int retVal = 0;
  if (!value) return retVal;
  noInterrupts();
  if (!lock) {
    retVal = 0;
  } else if (lock != value) {
    retVal = 1;
  } else {
    retVal = 2;
  }
  interrupts();
  return retVal;
}