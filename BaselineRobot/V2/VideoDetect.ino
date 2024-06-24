/* This file contains routines that provide feedback based on the video content
from the camera. None of this is intended to be a sophisticated image analysis but a 
group of routines that enable basic colour and shape detection, line following and obstacle avoidance. */

/* All routines expect to operate in RGB mode on the camera with low resolution
*/
struct blobLocation {
  long relativeX;
  long relativeY;
};


// *********************************************************************************************
// convert x and y co=ordinates into frame buffer offset
// *********************************************************************************************
long convertFrame(long width, long height) {
  return ((guideFrame.width * height) + width) * guideFrame.bytes_per_pixel;
}


// *********************************************************************************************
// convert a pixel location and colour into a validation value, typically a magnitude
// *********************************************************************************************
long detectColour(long pixel, int colour) {
  long pixSum = 0;
  long red255 = 0;
  long green255 = 0;
  long blue255 = 0;
  switch (colour) {
    case BLACK:
      // black is calculated as sum of colours subtracted from max, will require a different threshold
      pixSum = 765 - (guideFrame.data[pixel] + guideFrame.data[pixel + 1] + guideFrame.data[pixel + 2]);
      break;
    case 11:
      // red calulated as red minus the other two colours
      pixSum = guideFrame.data[pixel + 2] * guideFrame.data[pixel + 2] - (guideFrame.data[pixel + 1] * guideFrame.data[pixel + 1] + guideFrame.data[pixel] * guideFrame.data[pixel]);
      pixSum = map(pixSum, 0, 65026, 0, 255);
      break;
    case RED:
      // red calulated as red minus the other two colours
      pixSum = (guideFrame.data[pixel + 2] + guideFrame.data[pixel + 2] - (guideFrame.data[pixel + 1] + guideFrame.data[pixel]));
      break;
    case ORANGE:
      pixSum = 255 - (abs(guideFrame.data[pixel + 2] - guideFrame.data[pixel + 1] * 2) * 2) - guideFrame.data[pixel] * 4;
      break;
    case YELLOW:
      // yellow calculated as max illumination minus the difference between red and green minus four times the blue illumination
      pixSum = guideFrame.data[pixel + 2] + guideFrame.data[pixel + 1] - guideFrame.data[pixel] - guideFrame.data[pixel];
      break;
    case GREEN:
      // green calculated as green minus the other two colours
      pixSum = (guideFrame.data[pixel + 1] + guideFrame.data[pixel + 1] - (guideFrame.data[pixel + 2] + guideFrame.data[pixel]));
      break;
    case BLUE:
      // blue calcuated as blue minus the other two colours
      pixSum = (guideFrame.data[pixel] + guideFrame.data[pixel] - (guideFrame.data[pixel + 1] + guideFrame.data[pixel + 2]));
      break;
    case TURQ:
      pixSum = 255 - (abs(guideFrame.data[pixel + 1] - guideFrame.data[pixel]) * 2) - guideFrame.data[pixel + 2] * 4;
      break;
    case VIOLET:
      pixSum = 255 - (abs(guideFrame.data[pixel + 2] - guideFrame.data[pixel]) * 2) - guideFrame.data[pixel + 1] * 4;
      break;
    case WHITE:
      // for white, all colours are summed, the threshold value must reflect that.
      // This was modified for piwars to make teh contrast as lareg as possible
      // it may be extended to other colur detections
      pixSum = guideFrame.data[pixel] + guideFrame.data[pixel + 1] + guideFrame.data[pixel + 2];
      pixSum = map(pixSum * pixSum, 0, 585226, 0, 255);
      break;
    default:
      break;
  }
  return pixSum;
}
// *********************************************************************************************
// Search for blob in frame buffer
// *********************************************************************************************


int writeMask(int colour) {
  // blobMask holds co-ordinates of section of frame buffer to be ignored, such a a part of the robot in frame
  // Set blobMask area to colour specified
  if (blobMask[1] > 0 && blobMask[3] > 0) {
    for (int pixelH = blobMask[0]; pixelH <= blobMask[1]; pixelH++) {
      for (int pixelW = blobMask[2]; pixelW <= blobMask[3]; pixelW++) {
        setPixel(convertFrame(pixelW, pixelH), colour);
      }
    }
  }
  return 0;
}

long findBlob(int colour) {
  // search frame and highlight all pixels matching search criteria
  // the objective is to identify blobs within the frame and provide basic info as to the centre of the blob and it's size

  Serial.printf("colour in findblob %d\n", colour);
  Serial.printf("Threshold %d\n", blobThreshold);
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;  // size of frame in bytes
  long pixel = 0;                                                                          // variable used throughout to give offset into frame of pixel of interest

  //Initialise blobinstances to high values. Frame expected to be less than 255 pixels wide or high
  // for (int i = 0; i < 20; i++) {
  //   for (int j = 0; j < 5; j++) {
  //     blobInstances[i][j] = 255;
  //   }
  // }

  int light = pixelLuminance();

  //Pre-process frame into pixels of interest and black. Pixels of interest identified by colour and threshold
  // blue pixel used as indicator that pixel is of interest
  //
  long pixTest = 0;  // test value of pixel

  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {
    // call main colour detect routine to give a colour value
    pixTest = detectColour(pixel, colour);

    if (pixTest > blobThreshold) {
      // blue is used as it doesn't require additional processing
      setPixel(pixel, BLUE);
    } else setPixel(pixel, BLACK);
  }

  // De-noise picture, only areas with adjacent pixels considered
  // pixels of interest identified by adding green
  //
  // int lookAhead = 20;                                            // number of pixels to examine ahead
  int lookAheadPixels = lookAhead * guideFrame.bytes_per_pixel;  // number of bytes within frame to examine ahead
  int frameLimit = bytesPerFrame - lookAheadPixels;              // margin to stop scan when close to end of frame
  int testPixel = 0;                                             // offset from current test pixel
  int pixelBreak = 0;
  for (int pixel = 0; pixel < frameLimit; pixel = pixel + guideFrame.bytes_per_pixel) {
    if (guideFrame.data[pixel]) {
      pixelBreak = 0;
      for (testPixel = pixel; testPixel < pixel + lookAheadPixels; testPixel = testPixel + guideFrame.bytes_per_pixel) {
        if (!guideFrame.data[testPixel]) {
          pixelBreak = 1;
          break;
        }
      }
      if (!pixelBreak) {
        // add green to pixel to indicate its of interest
        guideFrame.data[pixel + 1] = 255;
      }
    }
  }

  // blobMask holds co-ordinates of section of frame buffer to be ignored, such a a part of the robot in frame
  // Set blobMask area to zero if it hasn't already been done
  if (!execMask) {
    writeMask(BLACK);
  }


  // scan the image until all pixels have been examined

  int pixCol = 0;       // set to the X offset within the frame
  int pixLine = 0;      // set to the Y offset within the frame
  int pixMinX = 255;    // minimum X value detected
  int pixMaxX = 0;      // maximum X value detected
  int pixMinY = 255;    // minimum Y value detected
  int pixMaxY = 0;      // maximum Y value detected
  long pixelLimit = 0;  // end of line
  long pixLineCount = 0;
  pixel = 0;
  int oldPixLine = 0;      // temporary variable holding last Y value before scan line, to reinstate frame scan after blob detection
  int oldPixCol = 0;       // temporary variable holding last X value before scan line, to reinstate frame scan after blob detection
  long maxPixelCount = 0;  // maximum blob pixel count for this scan
  int maxBlobX = 0;        // x co-ordinates of largest blob detected
  int maxBlobY = 0;        // y co-ordinates of largest blob detected

  // search frame and highlight all pixels matching search criteria
  // store the match level in the blue pixel for further processing.
  long lineWidth = guideFrame.width * guideFrame.bytes_per_pixel;

  // loop through the entire frame examining all pixels
  // a count of the number of lines and columns examined is kept separately.
  for (long lineOffset = 0; lineOffset < bytesPerFrame; lineOffset = lineOffset + guideFrame.width * guideFrame.bytes_per_pixel) {
    pixCol = 0;  // reset pixCol for start of new line

    for (long colOffset = 0; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel) {
      pixel = lineOffset + colOffset;
      // if pixel doesn't have red set, ie not yet examined, process 
      if (!guideFrame.data[pixel + 2]) {
        // if the pixel is valid, ie GREEN set, then treat as part of a blob
        if (guideFrame.data[pixel + 1]) {
          // run scanLine until end of blob,
          oldPixLine = pixLine;
          oldPixCol = pixCol;
          pixelCount = 0;  // reset pixel count

          // create a loop which is valid while a scanned line contains valid pixels
          // this counts the number of valid pixels found in a blob to gauge its size.
          // and approximate centre of the blob is calculated based on maximum height and width
          while (scanBlob(pixLine, pixCol) && pixLine < guideFrame.height - 1) {
            // update min and max X
            if (pixColMinX < pixMinX) {
              pixMinX = pixColMinX;
            }
            if (pixColMaxX > pixMaxX) {
              pixMaxX = pixColMaxX;
            }
            // calculate next column to start in
            pixCol = pixColMinX + (pixColMaxX - pixColMinX) / 2;
            // bump pixLine
            pixLine++;
          }
          // store blob metrics and restart scan at start of blob
          if (pixelCount > maxPixelCount) {
            maxPixelCount = pixelCount;
            maxBlobX = pixMinX + (pixMaxX - pixMinX) / 2;
            maxBlobY = oldPixLine + (pixLine - oldPixLine) / 2;
             Serial.printf("Size = %d,Line = %d, Col = %d \n", maxPixelCount, maxBlobY, maxBlobX);
          }
          pixLine = oldPixLine;
          pixCol = oldPixCol;
          pixMinX = 255;  // minimum X value detected
          pixMaxX = 0;    // maximum X value detected
          pixMinY = 255;  // minimum Y value detected
          pixMaxY = 0;    // maximum Y value detected
                          // break from loop
        }
      }
      // set pixel red to show processed
      guideFrame.data[pixel + 2] = 255;
      pixCol++;
    }
    pixLine++;
  }
  lastBlobCount = maxPixelCount;
  if (lastBlobCount > 9999) {
    lastBlobCount = 9999;
  }
  lastBlobX = maxBlobX;
  lastBlobY = maxBlobY;
  if (debugSerial) { Serial.printf("Size = %d,Line = %d, Col = %d, Luminance = %d \n", lastBlobCount, lastBlobY, lastBlobX, light); }

  return 0;
}
//**************************************************************************************************
// the scanBlob routine examines a line of the frame buffer left and right from the starting
// column, flagging all pixels as examined, updating the min and max values as necessary and
// stopping when both left and right pixels from the starting position are non-valid.
// the routine returns 1 if a valid pixel has been found, otherwise 0 to indicate the end of a
// blob
// pixLine is the current line being scanned
// pixCol is the starting column
// pixColMinX is the minimum blob x position found in the line examined
// pixColMaxX is the maximum blob x position found in the line examined
// pixelCount is the total of pixels foun din the current blob
//**************************************************************************************************
int scanBlob(int pixLine, int pixCol) {
  long pixLineStart = pixLine * guideFrame.width * guideFrame.bytes_per_pixel;
  long pixLineEnd = pixLineStart + (guideFrame.width - 1) * guideFrame.bytes_per_pixel;
  long pixel = pixLineStart + pixCol * guideFrame.bytes_per_pixel;
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;  // size of frame in bytes
  // initialise min and max
  pixColMaxX = pixCol;
  pixColMinX = pixCol;
  // test if starting pixel is non-valid
  // this may not be a good test as this routine develops but its a start
  // a development of this will be to search the line below the previous one for valid pixels
  if (!guideFrame.data[pixel + 1] || guideFrame.data[pixel + 2]) {
    // set pixel red to show that it has been examined
    guideFrame.data[pixel + 2] = 255;
    return 0;
  }


  int pixColTemp = pixCol;
  // search line to the right from start point until non-valid pixel
  for (int linePixel = pixel; linePixel < pixLineEnd; linePixel = linePixel + guideFrame.bytes_per_pixel) {
    // check if pixel has been examined before, and if so, break from this part of line check
    if (guideFrame.data[linePixel + 2]) {
      break;
    }

    if (linePixel >= bytesPerFrame) {
      Serial.printf("Frame size exceeded %d col = %d line = %d \n", linePixel, pixCol, pixLine);
      break;
    }
    // set pixel red to show that it has been examined
    guideFrame.data[linePixel + 2] = 255;
    // check if this pixel is valid
    if (guideFrame.data[linePixel + 1]) {
      // bump pixel count
      pixelCount++;

      // if new edges of box around blob then update edges
      if (pixColMaxX < pixColTemp) {
        pixColMaxX = pixColTemp;
      }

      // bump the column number
      pixColTemp++;
      if (pixColTemp >= guideFrame.width) break;
      // if not valid then end of blob line, and break
    } else break;
  }

  // search line to the left from start point until non-valid pixel
  pixColTemp = pixCol - 1;
  pixel = pixel - guideFrame.bytes_per_pixel;
  if (pixColTemp >= 0) {
    for (int linePixel = pixel; linePixel > pixLineStart; linePixel = linePixel - guideFrame.bytes_per_pixel) {
      // check if pixel has been examined before, and if so, break from this part of line check
      if (guideFrame.data[linePixel + 2]) {
        break;
      }
      if (linePixel >= bytesPerFrame) {
        Serial.printf("Frame size exceeded %d col = %d line = %d \n", linePixel, pixCol, pixLine);
        break;
      }
      if (linePixel < 0) {
        Serial.printf("Frame size undershot %d col = %d line = %d \n", linePixel, pixCol, pixLine);
        break;
      }
      // set pixel red to show that it has been examined
      guideFrame.data[linePixel + 2] = 255;
      if (guideFrame.data[linePixel + 1]) {
        // bump pixel count
        pixelCount++;

        // if new edges of box around blob then update edges
        if (pixColMinX > pixColTemp) {
          pixColMinX = pixColTemp;
        }

        // set pixel red to show that it has been examined
        pixColTemp--;
        if (pixColTemp <= 0) break;
      } else break;
    }
  }
  return 1;
}
// *********************************************************************************************
// Update frame buffer with colour map of detected pixels
// *********************************************************************************************
long heatMap(long colour, int background) {
  // search frame and highlight all pixels matching search criteria
  // frame can be displayed as a heat map or processed for blob location
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  int pixSum = 0;
  int pixTest = 0;
  int pixValue = 0;
  int pixTestBlue = 0;
  int pixTestGreen = 0;
  int pixTestRed = 0;

  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {
    //pixTest = guideFrame.data[pixel + 2] + guideFrame.data[pixel + 1] + guideFrame.data[pixel];

    if (pixTestBlue < guideFrame.data[pixel]) {
      pixTestBlue = guideFrame.data[pixel];
      //pixTestBlue = pixValue;
    }
    if (pixTestGreen < guideFrame.data[pixel + 1]) {
      pixTestGreen = guideFrame.data[pixel + 1];
      //pixTestGreen = pixValue;
    }
    if (pixTestRed < guideFrame.data[pixel + 2]) {
      pixTestRed = guideFrame.data[pixel + 2];
      //pixTestRed = pixValue;
    }
  }
  Serial.printf("max pixel = %d red = %d green = %d blue = %d\n", pixValue, pixTestRed, pixTestGreen, pixTestBlue);

  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {


    switch (colour) {
      case RED:
        pixTest = (guideFrame.data[pixel + 2] * 2 - (guideFrame.data[pixel + 1] + guideFrame.data[pixel])) / 2;

        break;
      case YELLOW:
        pixTest = (guideFrame.data[pixel + 2] + guideFrame.data[pixel + 1] - guideFrame.data[pixel] * 2) / 2;

        break;
      case GREEN:
        pixTest = (guideFrame.data[pixel + 1] * 2 - (guideFrame.data[pixel + 2] + guideFrame.data[pixel])) / 2;
        break;
      case BLUE:
        pixTest = (guideFrame.data[pixel] * 2 - (guideFrame.data[pixel + 2] + guideFrame.data[pixel + 1])) / 2;
        break;
      default:
        return 1;  // if not a valid colour supplied
    }

    // if pixel may be on line, highlight pixel
    if (pixTest < 32 && !background) {
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

// *********************************************************************************************
// Update frame buffer with highlighted colour area
// *********************************************************************************************
long findSquare(long colour) {
  // search frame for a square hightlighted in a different colour
  // frame can be displayed as a heat map or processed for blob location
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  int pixSum = 0;
  int pixTest = 0;
  int pixValue = 0;
  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {
    pixTest = guideFrame.data[pixel + 2] + guideFrame.data[pixel + 1] + guideFrame.data[pixel];
    if (pixTest > pixValue) {
      pixValue = pixTest;
    }
  }
  pixValue = pixValue / 3;
  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {

    switch (colour) {
      case RED:
        pixTest = (guideFrame.data[pixel + 2] * 2 - (guideFrame.data[pixel + 1] + guideFrame.data[pixel])) / 2;
        break;
      case GREEN:
        pixTest = guideFrame.data[pixel + 1] * 2 - (guideFrame.data[pixel + 2] + guideFrame.data[pixel]);
        break;
      case BLUE:
        pixTest = guideFrame.data[pixel];
        break;
      default:
        return 1;  // if not a valid colour supplied
    }
    pixTest = map(pixTest, 0, pixValue, 0, 255);
    // if pixel may be on line, highlight pixel
    if (pixTest > 64) {
      setPixel(pixel, RED);
    } else setPixel(pixel, BLACK);
  }

  return 0;
}

// *********************************************************************************************
// Update frame buffer with highlighted colour areas, two or three colours
// *********************************************************************************************
long findColours(void) {
  // search frame for a square hightlighted in a differnet colour
  // frame can be displayed as a heat map or processed for blob location
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  int pixSum = 0;
  int pixTest = 0;
  int pixRed = 0;
  int pixGreen = 0;
  int pixBlue = 0;
  int pixValue = 0;
  int pixUse = 0;
  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {
    pixTest = guideFrame.data[pixel + 2] + guideFrame.data[pixel + 1] + guideFrame.data[pixel];
    if (pixTest > pixValue) {
      pixValue = pixTest;
    }
  }
  pixValue = pixValue / 3;
  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {

    pixRed = (guideFrame.data[pixel + 2] + guideFrame.data[pixel + 2] - (guideFrame.data[pixel + 1] + guideFrame.data[pixel])) / 2;
    pixGreen = (guideFrame.data[pixel + 1] + guideFrame.data[pixel + 1] - (guideFrame.data[pixel + 2] + guideFrame.data[pixel])) / 2;
    pixBlue = (guideFrame.data[pixel] + guideFrame.data[pixel] - (guideFrame.data[pixel + 2] + guideFrame.data[pixel + 1])) / 2;
    pixTest = pixRed;
    pixUse = RED;
    if (pixGreen > pixTest) {
      pixTest = pixGreen;
      pixUse = GREEN;
    }
    if (pixBlue > pixTest) {
      pixTest = pixBlue;
      pixUse = BLUE;
    }

    pixTest = map(pixTest, 0, pixValue, 0, 255);
    // if pixel may be on line, highlight pixel
    if (pixTest > 64) {
      setPixel(pixel, pixUse);
    } else setPixel(pixel, BLACK);
  }

  return 0;
}
// *********************************************************************************************
// Search for detected pixels
// *********************************************************************************************
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
// *********************************************************************************************
// Search for detected pixels
// *********************************************************************************************
int scanTest(long colour) {
  // search frame and highlight all pixels matching search criteria
  // store the match level in the blue pixel for further processing.
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  long lineWidth = guideFrame.width * guideFrame.bytes_per_pixel;
  long pixel = 0;
  int pixTest = 0;
  for (long lineOffset = 0; lineOffset < bytesPerFrame; lineOffset = lineOffset + guideFrame.width * guideFrame.bytes_per_pixel) {
    for (long colOffset = 0; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel) {
      pixel = lineOffset + colOffset;

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
      if (pixTest > 64) {
        setPixel(pixel, WHITE);
      }
      if (colOffset > 420) {
        setPixel(pixel, YELLOW);
      }
      if (lineOffset > 52480) {
        setPixel(pixel, GREEN);
      }
    }
  }
  return 0;
}
// *********************************************************************************************
// put central crosshairs on the frame with graduations
// *********************************************************************************************
int crossHairs(long colour) {
  // search frame and highlight all pixels matching search criteria
  // store the match level in the blue pixel for further processing.
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  long lineWidth = guideFrame.width * guideFrame.bytes_per_pixel;
  long pixel = 0;
  int centreLineX = guideFrame.height / 2;
  int centreOffsetX = centreLineX * lineWidth;
  int centreLineY = guideFrame.width / 2;
  int centreOffsetY = centreLineY * guideFrame.bytes_per_pixel;

  // Draw centre X line
  for (long colOffset = 0; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel) {
    pixel = centreOffsetX + colOffset;
    setPixel(pixel, colour);
  }
  // draw graduations from centre to right
  for (long colOffset = centreOffsetY; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel * 10) {
    for (long gradOffset = centreOffsetX - (lineWidth * 5); gradOffset <= centreOffsetX + (lineWidth * 5); gradOffset = gradOffset + lineWidth) {
      pixel = gradOffset + colOffset;
      setPixel(pixel, colour);
    }
  }
  // draw graduations from centre to left
  for (long colOffset = centreOffsetY; colOffset > 0; colOffset = colOffset - guideFrame.bytes_per_pixel * 10) {
    for (long gradOffset = centreOffsetX - (lineWidth * 5); gradOffset <= centreOffsetX + (lineWidth * 5); gradOffset = gradOffset + lineWidth) {
      pixel = gradOffset + colOffset;
      setPixel(pixel, colour);
    }
  }
  // Draw centre Y line
  for (long lineOffset = 0; lineOffset < bytesPerFrame; lineOffset = lineOffset + lineWidth) {
    pixel = lineOffset + centreOffsetY;
    setPixel(pixel, colour);
  }
  // draw graduations from centre to bottom
  for (long lineOffset = centreOffsetX; lineOffset < bytesPerFrame; lineOffset = lineOffset + lineWidth * 10) {
    for (long gradOffset = centreOffsetY - (guideFrame.bytes_per_pixel * 5); gradOffset <= centreOffsetY + (guideFrame.bytes_per_pixel * 5); gradOffset = gradOffset + guideFrame.bytes_per_pixel) {
      pixel = gradOffset + lineOffset;
      setPixel(pixel, colour);
    }
  }
  // draw graduations from centre to top
  for (long lineOffset = centreOffsetX; lineOffset > 0; lineOffset = lineOffset - lineWidth * 10) {
    for (long gradOffset = centreOffsetY - (guideFrame.bytes_per_pixel * 5); gradOffset <= centreOffsetY + (guideFrame.bytes_per_pixel * 5); gradOffset = gradOffset + guideFrame.bytes_per_pixel) {
      pixel = gradOffset + lineOffset;
      setPixel(pixel, colour);
    }
  }

  return 0;
}
// *********************************************************************************************
// put pixel count graduation ticks arount the outside of the frame
// *********************************************************************************************
int percentGrads(long colour) {
  // search frame and highlight all pixels matching search criteria
  // store the match level in the blue pixel for further processing.
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  long lineWidth = guideFrame.width * guideFrame.bytes_per_pixel;
  long pixel = 0;
  int centreLineX = guideFrame.height / 2;
  int centreOffsetX = centreLineX * lineWidth;
  int centreLineY = guideFrame.width / 2;
  int centreOffsetY = centreLineY * guideFrame.bytes_per_pixel;
  int widthGrads = guideFrame.width / 10;
  int heightGrads = guideFrame.height / 10;

  // Draw centre X line
  // for (long colOffset = 0; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel) {
  //   pixel = centreOffsetX + colOffset;
  //   setPixel(pixel, colour);
  // }
  // draw graduations across the top
  for (long colOffset = widthGrads * guideFrame.bytes_per_pixel; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel * widthGrads) {
    for (long gradOffset = 0; gradOffset <= (lineWidth * 10); gradOffset = gradOffset + lineWidth) {
      pixel = gradOffset + colOffset;
      setPixel(pixel, colour);
    }
  }
  // draw graduations across the bottom
  for (long colOffset = widthGrads * guideFrame.bytes_per_pixel; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel * widthGrads) {
    for (long gradOffset = (guideFrame.height - 10) * lineWidth; gradOffset < bytesPerFrame; gradOffset = gradOffset + lineWidth) {
      pixel = gradOffset + colOffset;
      setPixel(pixel, colour);
    }
  }
  // Draw centre Y line
  // for (long lineOffset = 0; lineOffset < bytesPerFrame; lineOffset = lineOffset + lineWidth) {
  //   pixel = lineOffset + centreOffsetY;
  //   setPixel(pixel, colour);
  // }
  // draw graduations down left side
  for (long lineOffset = lineWidth * heightGrads; lineOffset < bytesPerFrame; lineOffset = lineOffset + lineWidth * heightGrads) {
    for (long gradOffset = 0; gradOffset <= (guideFrame.bytes_per_pixel * 10); gradOffset = gradOffset + guideFrame.bytes_per_pixel) {
      pixel = gradOffset + lineOffset;
      setPixel(pixel, colour);
    }
  }
  // draw graduations down right side
  for (long lineOffset = lineWidth * heightGrads; lineOffset < bytesPerFrame; lineOffset = lineOffset + lineWidth * heightGrads) {
    for (long gradOffset = lineWidth - (guideFrame.bytes_per_pixel * 10); gradOffset <= lineWidth; gradOffset = gradOffset + guideFrame.bytes_per_pixel) {
      pixel = gradOffset + lineOffset;
      setPixel(pixel, colour);
    }
  }

  return 0;
}
// *********************************************************************************************
// put % graduation ticks arount the outside of the frame
// *********************************************************************************************
int graduations(long colour) {
  // search frame and highlight all pixels matching search criteria
  // store the match level in the blue pixel for further processing.
  long bytesPerFrame = guideFrame.height * guideFrame.width * guideFrame.bytes_per_pixel;
  long lineWidth = guideFrame.width * guideFrame.bytes_per_pixel;
  long pixel = 0;
  int centreLineX = guideFrame.height / 2;
  int centreOffsetX = centreLineX * lineWidth;
  int centreLineY = guideFrame.width / 2;
  int centreOffsetY = centreLineY * guideFrame.bytes_per_pixel;

  // Draw centre X line
  // for (long colOffset = 0; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel) {
  //   pixel = centreOffsetX + colOffset;
  //   setPixel(pixel, colour);
  // }
  // draw graduations across the top
  for (long colOffset = 10 * guideFrame.bytes_per_pixel; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel * 10) {
    for (long gradOffset = 0; gradOffset <= (lineWidth * 10); gradOffset = gradOffset + lineWidth) {
      pixel = gradOffset + colOffset;
      setPixel(pixel, colour);
    }
  }
  // draw graduations across the bottom
  for (long colOffset = 10 * guideFrame.bytes_per_pixel; colOffset < lineWidth; colOffset = colOffset + guideFrame.bytes_per_pixel * 10) {
    for (long gradOffset = (guideFrame.height - 10) * lineWidth; gradOffset < bytesPerFrame; gradOffset = gradOffset + lineWidth) {
      pixel = gradOffset + colOffset;
      setPixel(pixel, colour);
    }
  }
  // Draw centre Y line
  // for (long lineOffset = 0; lineOffset < bytesPerFrame; lineOffset = lineOffset + lineWidth) {
  //   pixel = lineOffset + centreOffsetY;
  //   setPixel(pixel, colour);
  // }
  // draw graduations down left side
  for (long lineOffset = lineWidth * 10; lineOffset < bytesPerFrame; lineOffset = lineOffset + lineWidth * 10) {
    for (long gradOffset = 0; gradOffset <= (guideFrame.bytes_per_pixel * 10); gradOffset = gradOffset + guideFrame.bytes_per_pixel) {
      pixel = gradOffset + lineOffset;
      setPixel(pixel, colour);
    }
  }
  // draw graduations down right side
  for (long lineOffset = lineWidth * 10; lineOffset < bytesPerFrame; lineOffset = lineOffset + lineWidth * 10) {
    for (long gradOffset = lineWidth - (guideFrame.bytes_per_pixel * 10); gradOffset <= lineWidth; gradOffset = gradOffset + guideFrame.bytes_per_pixel) {
      pixel = gradOffset + lineOffset;
      setPixel(pixel, colour);
    }
  }

  return 0;
}
// *********************************************************************************************
// Updates pixel at location with selected colour
// *********************************************************************************************
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

// *********************************************************************************************
// Change camera settings for guidance
// *********************************************************************************************
void setUpCamera(void) {
  // changes camera settings to standardise the detection operation
}

// *********************************************************************************************
// Restore saved camera settings
// *********************************************************************************************
void resetCamera(void) {
  // restores camera to previous configuration
}

// *********************************************************************************************
// save camera settings before modification
// *********************************************************************************************
void saveCamera(void) {
  // saves camera settings for restore later
}

// *********************************************************************************************
// Instigate routine to locate line position at top of frame
// *********************************************************************************************
long findLineTop(long colour) {
  // returns the relative position of a coloured line in the top view of the camera
  int retval = findLine(lineInstancesT, colour, 12, 0, 143, lineThresholdT, 5, 7, 1);

  return 0;
}

// *********************************************************************************************
// Instigate routine to locate line position in middle of frame
// *********************************************************************************************
long findLineCentre(long colour) {
  // returns the relative position of a coloured line in the Centre view of the camera
  //  long pixLuminance = pixelLuminance(0);
  //Serial.println(pixelLuminance(0));
  int retval = findLine(lineInstancesC, colour, 60, 0, 143, lineThresholdC, 5, 7, 1);

  return 0;
}

// *********************************************************************************************
// Instigate routine to locate line position at bottom of frame
// *********************************************************************************************
long findLineBottom(long colour) {
  // returns the relative position of a coloured line in the Bottom view of the camera
  //Pre-process frame to get average brightness
  //

  int retval = findLine(lineInstancesB, colour, 130, 0, 143, lineThresholdB, 5, 7, 1);

  return 0;
}

// *********************************************************************************************
// Routine to take an average of the pixel values
// *********************************************************************************************
int pixelLuminance(void) {
  // get luminance value from camera
  sensor_t *s = esp_camera_sensor_get();
  s->set_reg(s, 0xff, 0xff, 0x01);  //banksel

  int pixLuminance = s->get_reg(s, 0x2f, 0xff);
  return pixLuminance;
}

// *********************************************************************************************
// Preprocess frame image
// *********************************************************************************************
long pixelPreprocess(int type1, int type2, int type3) {
  long bytesPerFrame = guideFrame.width * guideFrame.height * guideFrame.bytes_per_pixel;
  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {
    switch (type1) {
      case 1:
        guideFrame.data[pixel] = 255 - guideFrame.data[pixel];
        guideFrame.data[pixel + 1] = 255 - guideFrame.data[pixel + 1];
        guideFrame.data[pixel + 2] = 255 - guideFrame.data[pixel + 2];
      default:
        break;
    }
  }
  return 0;
}

// *********************************************************************************************
// Invert
// *********************************************************************************************
long pixelInvert(void) {
  long bytesPerFrame = guideFrame.width * guideFrame.height * guideFrame.bytes_per_pixel;
  for (int pixel = 0; pixel < bytesPerFrame; pixel = pixel + guideFrame.bytes_per_pixel) {
    guideFrame.data[pixel] = 255 - guideFrame.data[pixel];
    guideFrame.data[pixel + 1] = 255 - guideFrame.data[pixel + 1];
    guideFrame.data[pixel + 2] = 255 - guideFrame.data[pixel + 2];
  }
  return 0;
}

// *********************************************************************************************
// Routine to locate line position
// *********************************************************************************************
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
  long pixSum = 0;                        // total of pixel values
  int colOffset = 0;                      // offset from start of column
  int colSum = 0;                         // column total of pixel values
  int colThreshold = column * threshold;  // threshold for a column
  int validCount = 0;                     // record of consecutive valid pixels
  int lineCount = 0;                      // number of lines, value stored in first entry of lineInstances
  int newCount = 1;                       // temporary value stored while line being verified

  // Checks
  if (startLine + column > guideFrame.height) return 2;

  //long pixLuminance = pixelLuminance(0);
  // Serial.println(pixelLuminance(0));
  for (int i = 0; i < 7; i++) {
    lineInstances[i] = 0;
  }


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

      pixOffset = scanCol + colOffset;           // using addition to avoid overhead of mulitplication
      pixSum = detectColour(pixOffset, colour);  //- pixLuminance ;

      colSum = colSum + pixSum;

      // if pixel may be on line, highlight pixel
      if (update && pixSum >= threshold) {
        setPixel(pixOffset, GREEN);
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


// *********************************************************************************************
// Test routine to print line position
// *********************************************************************************************
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
// *********************************************************************************************
// get frame buffer new
// *********************************************************************************************
int videoProcessing(void) {
  camera_fb_t *webBuffer = NULL;
  frameAvailable = 2;  // signal frame processing in progress
  webBuffer = esp_camera_fb_get();
  out_width = webBuffer->width;
  out_height = webBuffer->height;
  out_len = webBuffer->width * webBuffer->height * 3;
  if (out_len >= 100000) {
    if (debugSerial) { Serial.printf("Frame too big to copy for processing %d \n", out_len); }
    frameAvailable = 3;  // signal frame processing in error
  } else {
    out_buf = (uint8_t *)malloc(out_len);
    if (!out_buf) {
      if (debugSerial) { Serial.println("Conversion buffer allocation for processing failed"); }
      frameAvailable = 3;  // signal frame processing in error
    } else {
      bool rgb888_convert = fmt2rgb888(webBuffer->buf, webBuffer->len, webBuffer->format, out_buf);
      guideFrame.data = out_buf;
      guideFrame.width = webBuffer->width;
      guideFrame.height = webBuffer->height;
      guideFrame.format = FB_BGR888;
      guideFrame.bytes_per_pixel = 3;

      esp_camera_fb_return(webBuffer);
      webBuffer = NULL;

      if (!rgb888_convert) {
        free(out_buf);
        if (debugSerial) { Serial.println("Conversion to rgb888 for processing failed"); }
        frameAvailable = 3;  // signal frame processing in error
        return 1;
      } else frameAvailable = 1;  // signal frame processing successful
    }
  }


  return 0;
}

// *********************************************************************************************
// Execute guidance routines
// *********************************************************************************************
int procGuidance(int type) {
  // Core video guidance routines for standalone running
  // This routine is called on demand to return video guidance information
  // call types are 1 - get new video frame
  int status = 0;
  if (frameAvailable == 5) {
    frameAvailable = 0;
    if (guideFrame.data) {
      free(guideFrame.data);
      guideFrame.data = NULL;
    }
  }
  switch (type) {
    case 1:
      //Serial.println("getting frame");
      if (!frameAvailable) {
        status = videoProcessing();
      }
      //Serial.println(frameAvailable);
      break;
    // if frame available, process selected detection routines
    case 2:
      // Serial.println("analysing frame");
      if (frameAvailable == 1) {
        //Serial.println("analysing frame");
        // call line detection processing. This is currently only setup for white line on black background
        // three frame positions are provided here, top, centre and bottom, and three 'lines' can detected and reported on in each
        // the position of top, centre and bottom can be defined in commands to suit the camera
        // by default, only the centre position is updated, selecting BOTTOM updates both centre and bottom, and selecting
        // TOP updates top, centre and bottom
        if (execLine) {
          switch (execLineProcessing) {
            case 1:
              status = findLineTop(colourDetect);
            case 2:
              status = findLineBottom(colourDetect);
            default:
              status = findLineCentre(colourDetect);
          }
        }
        // call blob detection processing
        // the initial reporting is targeted on the centre of the largest blob detected
        else if (execBlob) {

          status = findBlob(colourDetect);
        }
      }
      break;

    case 3:
      // delete guidance frame

      //Serial.println("deleting frame");
      frameAvailable = 0;
      if (guideFrame.data) {
        free(guideFrame.data);
        guideFrame.data = NULL;
      }
      break;
    default:
      break;
  }
  return 0;
}

int calibrate(int type, int colour) {
  // this routine measures the luminance of the image and adjusts the exposure control
  int retVal = 1;
  // calTimer = millis();  // get start time
  // switch (type) {
  //   case 0:
  //     sensor_t *s = esp_camera_sensor_get();
  //     s->set_reg(s, 0xff, 0xff, 0x01);  //banksel
  //     int calTemp = s->get_reg(s, 0x2f, 0xff);
  //     // check if luminance has changes since last adjustment
  //     if ((calTemp <= calLuminance + 10) && (calTemp >= calLuminance -10)) {
  //       retVal = 1;
  //     } else if (calTemp < 100) {
  //       // increase exposure
  //     } else if (calTemp > 100) {}

  //   default:
  //}


  return retVal;
}