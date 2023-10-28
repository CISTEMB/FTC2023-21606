// Drive forward to in between the three lines
driveForward(3);
// Scan distance sensors for team prop
int distanceSensor = scanDistanceSensors();
// Turn to face the team prop
turnToFaceTeamProp(distanceSensor);
// Turn slightly further
turn(1); //degree
// And drop a pixel on the same line as the team prop
dropPixel();
// Then turn to face the backdrop
turnToFaceBackdrop(); // Somewhere from 0-180 degrees
// And drive forward until the backdrop is detected
driveForwardUntilBackdrop();
// Then strafe until the distance sensor closest to the wall is reading the correct value for the line previously detected
strafeUntilLine();
// Then reach the arm upward at the correct angle
reachArmUpward();
// And drop the pixel
dropPixel();
// Then strafe over to the parking zone
strafeToParkingZone();