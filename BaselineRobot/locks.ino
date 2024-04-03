// lock routines to ensure that script processing runs in harmony with input commands from PS3 or Webpages
// this will be recoded as a class in the future
// *********************************************************************************************
// lockset sets a lock
// *********************************************************************************************
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

// *********************************************************************************************
// lockunset unsets a lock
// *********************************************************************************************
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
// *********************************************************************************************
// locktest tets a lock
// *********************************************************************************************
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