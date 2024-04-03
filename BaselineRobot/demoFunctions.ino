/* This file conntains routines to demonstrate the autonomous functions of the L0cost robot controller
when running stand alone. This mainly comes down to line following, robot object following and 
pan & tilt object following. 

The routines are called from the main loop routine without contect and must maintain their own state

These routines are rudimentary and are at the 'get you going' stage
*/

int lineFollower(void) {
  /* This routine gets the last line following guidance information and uses it to create and send 
  motor commands to the destination set by the LD commands, default Local, effectively implementing a line tracking function
  This uses a PID routine with default Kp, Ki and Kd values but these can all be set by the user  */

  /* there are three states possible, static, line following and calibration. when line following is first or subsequently
  selected, the robot waits for a change of state. The calibrate command runs the calibration routine until complete.
  The follow command is ignored until the calibrate command is complete, after which it runs continuously until
  another command which negates it is received, usually a STOP command. */
  switch (demoStatus) {
    case 0:
    break;
    case 1:
    break;
    case 2:
    break;
    default:
    execDemo = 0;
  }
}
int blobFollower(void) {
  /* This routine gets the last blob location information and uses it to create and send 
  motor commands to the destination set by the LD commands, default Local, allowing a robot to drive around locating
  objects. This uses a PID routine with default Kp, Ki and Kd values but these can all be set by the user */

    /* there are three states possible, static, blob following and calibration. when blob following is first or subsequently
  selected, the robot waits for a change of state. The calibrate command runs the calibration routine until complete.
  The follow command is ignored until the calibrate command is complete, after which it runs continuously until
  another command which negates it is received, usually a STOP command. */
}
int targetFollower(void) {
  /* This routine gets the last blob location information information and uses it to create and send 
  servo commands to the destination set by the LD commands, default Local, effectively implementing an
  aiming system. This uses a PID routine with default Kp, Ki and Kd values but these can all be set by the user */

    /* there are three states possible, static, target following and calibration. when target following is first or subsequently
  selected, the robot waits for a change of state. The calibrate command runs the calibration routine until complete.
  The follow command is ignored until the calibrate command is complete, after which it runs continuously until
  another command which negates it is received, usually a STOP command. */
}
