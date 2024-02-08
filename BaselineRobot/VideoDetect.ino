/* This file contains routines that provide feedback based on the video content
from the camera. None of this is intended to be a sophisticated image analysis but a 
group of routines that enable basic colour and shape detection, line following and obstacle avoidance. */

/* All routines expect to operate in RGB mode on the camera with low resolution
*/
struct blobLocation {
  long relativeX;
  long relativeY;
};
long findBlobInstances(long colour) {
  // call findBlob routine
  long blobInstances[13] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int retval = findBlob(blobInstances, colour, 0, 300, 5, 5, 1);
  return 0;
}
long findBlob(long blobInstances[], long colour, long startLine, long threshold, long adjacent, long column, long update) {
  // search frame and highlight all pixels matching search criteria
  // the objective is to identify blobs within the frame and provide basic info as to the centre of the blob and it's size
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  int pixSum = 0;

  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {

    switch (colour) {
      case BLACK:
      case RED:
        if (guideFrame.data[pixel] < 50 && guideFrame.data[pixel + 1] < 50 && guideFrame.data[pixel + 2] > 100) {
          pixSum = threshold + 1;
        } else pixSum = 0;
        break;
      case ORANGE:
      case YELLOW:
      case GREEN:
      case BLUE:
      case VIOLET:
      case WHITE:
        // for white, all colours are summed, the threshold value must reflect that.
        pixSum = guideFrame.data[pixel] + guideFrame.data[pixel + 1] + guideFrame.data[pixel + 2];
        break;
      default:
        return 1;  // if not a valid colour supplied
    }
    // if pixel may be on line, highlight pixel
    if (update) {
      if (pixSum >= threshold) {
        setPixel(pixel, YELLOW);
      } else {
        // setPixel(pixel,RED);
      }
    }
  }
  return 0;
}

long heatMap(long colour) {
  // search frame and highlight all pixels matching search criteria
  // frame can be displayed as a heat map or processed for blob location
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  int pixSum = 0;
  int pixTest = 0;

  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {

    switch (colour) {
      case RED:
        pixTest = guideFrame.data[pixel + 2] * 2 - (guideFrame.data[pixel + 1] + guideFrame.data[pixel]);
        break;
      case GREEN:
        pixTest = guideFrame.data[pixel + 1];
        break;
      case BLUE:
        pixTest = guideFrame.data[pixel];
        break;
      default:
        return 1;  // if not a valid colour supplied
    }
    // if pixel may be on line, highlight pixel
    if (pixTest < 32) {
      setPixel(pixel, BLACK);
    } else if (pixTest < 64) {
      setPixel(pixel, VIOLET);
    } else if (pixTest < 96) {
      setPixel(pixel, BLUE);
    } else if (pixTest < 128) {
      setPixel(pixel, GREEN);
    } else if (pixTest < 160) {
      setPixel(pixel, YELLOW);
    } else if (pixTest < 192) {
      setPixel(pixel, ORANGE);
    } else if (pixTest < 224) {
      setPixel(pixel, RED);
    } else setPixel(pixel, WHITE);
  }

  return 0;
}
long heatTest(long colour) {
  // search frame and highlight all pixels matching search criteria
  // store the match level in the blue pixel for further processing.
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  int pixSum = 0;
  int pixTest = 0;

  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {

    switch (colour) {
      case RED:
        pixTest = guideFrame.data[pixel + 2] * 2 - (guideFrame.data[pixel + 1] + guideFrame.data[pixel]);
        break;
      case GREEN:
        pixTest = guideFrame.data[pixel + 1];
        break;
      case BLUE:
        pixTest = guideFrame.data[pixel];
        break;
      default:
        return 1;  // if not a valid colour supplied
    }
    // if pixel may be on line, highlight pixel
    if (pixTest > 255) pixTest = 255;
    guideFrame.data[pixel + 2] = pixTest;
    guideFrame.data[pixel + 1] = 0;
    guideFrame.data[pixel] = 0;
  }

  return 0;
}
int setPixel(long location, int colour) {
  switch (colour) {
    case BLACK:
      guideFrame.data[location] = 0;
      guideFrame.data[location + 1l] = 0;
      guideFrame.data[location + 2l] = 0;
      break;
    case RED:
      guideFrame.data[location] = 0;
      guideFrame.data[location + 1l] = 0;
      guideFrame.data[location + 2l] = 255;
      break;
    case ORANGE:
      guideFrame.data[location] = 0;
      guideFrame.data[location + 1l] = 128;
      guideFrame.data[location + 2l] = 255;
      break;
    case YELLOW:
      guideFrame.data[location] = 0;
      guideFrame.data[location + 1l] = 255;
      guideFrame.data[location + 2l] = 255;
      break;
    case GREEN:
      guideFrame.data[location] = 0;
      guideFrame.data[location + 1l] = 255;
      guideFrame.data[location + 2l] = 0;
      break;
    case BLUE:
      guideFrame.data[location] = 255;
      guideFrame.data[location + 1l] = 0;
      guideFrame.data[location + 2l] = 0;
      break;
    case VIOLET:
      guideFrame.data[location] = 255;
      guideFrame.data[location + 1l] = 0;
      guideFrame.data[location + 2l] = 128;
      break;
    case WHITE:
      guideFrame.data[location] = 255;
      guideFrame.data[location + 1l] = 255;
      guideFrame.data[location + 2l] = 255;
      break;
    default:
      return 1;  // if not a valid colour supplied
  }
}
void setUpCamera(void) {
  // changes camera settings to standardise the detection operation
}

void resetCamera(void) {
  // restores camera to previous configuration
}

void saveCamera(void) {
  // saves camera settings for restore later
}

long findLineTop(long colour) {
  // returns the relative position of a coloured line in the top view of the camera
 // long lineInstancesT[7] = { 0, 0, 0, 0, 0, 0, 0};
  int retval = findLine(lineInstancesT, WHITE, 0, 0, 143, lineThresholdT, 5, 5, 1);
  // if (lineInstances[0] != 0) {
  // Serial.printf("Lines : %d ", lineInstances[0]);
  // Serial.print("1,");
  // if (lineInstancesT[0] != 0) {
  //   for (int i = 1; i <= lineInstancesT[0]; i++) {
  //     Serial.printf("%d,%d", lineInstancesT[i], lineInstancesT[i + 1]);
  //   }
  // } else {
  //   Serial.print("0,0");
  // }
  //Serial.println("");
  //}
  return 0;
}

long findLineCentre(long colour) {
  // returns the relative position of a coloured line in the Centre view of the camera
 // long lineInstances[7] = { 0, 0, 0, 0, 0, 0, 0};
  int retval = findLine(lineInstancesC, WHITE, 69, 0, 143, lineThresholdC, 5, 5, 1);
  //if (lineInstances[0] != 0) {
  // Serial.print("2,");
  // //Serial.printf("Lines : %d ", lineInstances[0]);
  // if (lineInstancesB[0] != 0) {
  //   for (int i = 1; i <= lineInstancesC[0]; i++) {
  //     Serial.printf("%d,%d", lineInstancesC[i], lineInstancesC[i + 1]);
  //   }
  // } else {
  //   Serial.print("0,0");
  // }
  //Serial.println("");
  //}
  return 0;
}

long findLineBottom(long colour) {
  // returns the relative position of a coloured line in the Bottom view of the camera
  //long lineInstances[7] = { 0, 0, 0, 0, 0, 0, 0};
  int retval = findLine(lineInstancesB, WHITE, 138, 0, 143, lineThresholdB, 5, 5, 1);
  //if (lineInstances[0] != 0) {
  // Serial.print("3,");
  // // Serial.printf("Lines : %d ", lineInstances[0]);
  // if (lineInstancesB[0] != 0) {
  //   for (int i = 1; i <= lineInstancesB[0]; i++) {
  //     Serial.printf("%d,%d", lineInstancesB[i], lineInstancesB[i + 1]);
  //   }
  // } else {
  //   Serial.print("0,0");
  // }
  // // Serial.println("");
  // //}
  return 0;
}

long findLine(long lineInstances[], long colour, long startLine, long startCol, long endCol, long threshold, long adjacent, long column, long update) {
  /* 
  colour is a number representing the colour of line being detected
  startLine is the first line of the frame buffer to be searched
  threshold is the value for an individual pixel to be accepted as valid
  adjacent is the number of adjacent pixels to be valid that constitute the start of a line
  column is the number of successive lines in the frame buffer to be included.
  update is a value which indicates that the frame buffer is to be updated to show valid pixels

  The frame buffer is scanned from the left to right 
  Each column of selected pixels is scanned, added to a running total for the column and if valid, are updated
  If a column sum is greater than the threshold (threshold x number of pixels) then it is flagged as a start
  of a line or recorded as an update to the adjacent line count'
  When the number of adjacent lines which are valid is equal to the adjacent line value, the start of line is flagged.
  When a column is flagged as invalid, the adjacent line count is decremented.
  When the adjacent line count reaches zero, the end of line is flagged.  
  The process is repeated across the width of the frame buffer, possibly generating several line instances, max 6)
  It is up to the requesting software to interpret these.

  return values
  0 OK
  1 invalid colour value
  2 invalid startline
  3 two many lines?

  */
  int instOffset = 1;                     // pointer to offset in instances. Instance 0 is number of start/end pairs
  int pixOffset = 0;                      // offset of pixel from startline
  int pixSum = 0;                         // total of pixel values
  int colOffset = 0;                      // offset from start of column
  int colSum = 0;                         // column total of pixel values
  int colThreshold = column * threshold;  // threshold for a column
  int validCount = 0;                     // record of consecutive valid pixels
  int lineCount = 0;                      // number of lines, value stored in first entry of lineInstances
  int newCount = 1;                       // temporary value stored while line being verified

  // Checks
  if (startLine + column > guideFrame.height) return 2;

  // setup scanning loop
  long bytesPerRow = guideFrame.width * guideFrame.bytes_per_pixel;
  long startPos = startLine * bytesPerRow;  //calculate the position in the frame buffer where this line starts
  long endPos = startPos + bytesPerRow;     // calculate end position
  long scanCol = 0;
  long retCol = 0;

  for (scanCol = startPos; scanCol < endPos; scanCol = scanCol + guideFrame.bytes_per_pixel) {  // cycle through the columns

    colOffset = 0;
    colSum = 0;

    for (int columnRow = 0; columnRow < column; columnRow++) {

      pixOffset = scanCol + colOffset;  // using addition to avoid overhead of mulitplication

      switch (colour) {
        case BLACK:
        case RED:
        case ORANGE:
        case YELLOW:
        case GREEN:
        case BLUE:
        case VIOLET:
        case WHITE:
          // for white, all colours are summed, the threshold value must reflect that.
          pixSum = guideFrame.data[pixOffset] + guideFrame.data[pixOffset + 1] + guideFrame.data[pixOffset + 2];
          break;
        default:
          return 1;  // if not a valid colour supplied
      }

      colSum = colSum + pixSum;

      // if pixel may be on line, highlight pixel
      if (update && pixSum >= threshold) {
        guideFrame.data[pixOffset] = 0;
        guideFrame.data[pixOffset + 1] = 255;  // sets green to max
        guideFrame.data[pixOffset + 2] = 0;
      }
      colOffset = colOffset + bytesPerRow;  // check this value . calculate new offset using addition
    }
    // if colSum greater than threshold, then process a possible start of line
    if (colSum >= colThreshold) {
      if (validCount == 0) {
        lineInstances[instOffset] = (retCol * 100) / guideFrame.width;  // if this is the first valid pixel, record the value
      }
      validCount++;
      if (validCount >= adjacent) {
        validCount = adjacent;
        if (newCount != lineInstances[0]) {  // if lineCount ne to next line count then make them equal
          lineInstances[0] = newCount;
        }
      }
    }

    // else process a possible end of line
    else {
      if (validCount == adjacent) {
        lineInstances[instOffset + 1] = (retCol * 100) / guideFrame.width;  // if this is the first non-valid pixel, record the value
        // lineInstances[0]++;
      }
      validCount--;
      if (validCount <= 0) {
        validCount = 0;
        if (newCount == lineInstances[0]) {  // if line end confirmed and lineCount eq newLine then inc newLine value
          newCount++;
          if (newCount >= 6) {
            return 3;
          }
          instOffset = instOffset + 2;  // and set new instances value
          if (instOffset >= 6) {
            break;
          }
        }
      }
    }
    retCol++;  // increment the column to report
  }
  if (validCount == adjacent) {
    lineInstances[instOffset + 1] = (retCol * 100) / guideFrame.width;
  }
  return 0;
}


// long findLineCentre(long colour) {
//   // returns the relative position of a coloured line in the centre view of the camera
//   for (long i = 35376; i <= 40653; i = i + 3) {  // 5280
//     if (guideFrame.data[i] + guideFrame.data[i + 1] + guideFrame.data[i + 2] > lineLuminence) {
//       guideFrame.data[i] = 0;
//       guideFrame.data[i + 1] = 255;
//       guideFrame.data[i + 2] = 0;
//     }
//   }
//   return 0;
// }



// long findLineBottom(long colour) {
//   // returns the relative position of a coloured line in the bottom view of the camera
//   for (long i = 69696; i <= 76029; i = i + 3) {  // 5280
//     if (guideFrame.data[i] + guideFrame.data[i + 1] + guideFrame.data[i + 2] > lineLuminence) {
//       guideFrame.data[i] = 0;
//       guideFrame.data[i + 1] = 255;
//       guideFrame.data[i + 2] = 0;
//     }
//   }
//   return 0;
// }

int printLinePos(long colour) {
  if (!printData) {
    Serial.printf("Width %d Height %d \n", guideFrame.width, guideFrame.height);
    printData = 1;
    int offset = 0;

    for (int i = 1; i < 170; i++) {
      Serial.printf("Red %d Green %d Blue %d \n", guideFrame.data[i * 3], guideFrame.data[(i * 3 + 1)], guideFrame.data[(i * 3 + 2)]);
    }
  }
}
