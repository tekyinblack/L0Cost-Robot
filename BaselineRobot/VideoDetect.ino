/* This file contains routines that provide feedback based on the video content
from the camera. None of this is intended to be a sophisticated image analysis but a 
group of routines that enable basic colour and shape detection, line following and obstacle avoidance. */

/* All routines expect to operate in RGB mode on the camera with low resolution
*/
struct blobLocation {
  long relativeX;
  long relativeY;
};
void setUpCamera(void) {
  // changes camera settings to standardise the detection operation
}

void resetCamera(void) {
  // restores camera to previous configuration
}

void saveCamera(void) {
  // saves camera settings for restore later
}

long findLineCentre(long colour) {
  // returns the relative position of a coloured line in the centre view of the camera
}

long findLineTop(long colour) {
  // returns the relative position of a coloured line in the top view of the camera
}

blobLocation scanBlob(blobLocation start, long colour) {
  // returns the relative position of a coloured blob in the camera image
  // beginning the search at position start
}