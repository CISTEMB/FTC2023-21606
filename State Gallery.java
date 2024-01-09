/*
Gallery of standard, full featured, states
 */

/*
			case GENERIC_STATE: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
				// Things to do first time in a state
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();   // 
                    speedStrafe(-.05); 
                    InitDriveState = false;
				// Reasons to leave state
                } else if (gotLine()) {  	
                    stopDrive();
                    newDriveState(DriveState.LEFT_MOVE_BACK_AGAIN);
                    
                } else if (gotStrafeDistance(-10, false)){ // todo make 40
                    newDriveState(DriveState.END);
                }
				// Things to do every time the state is looped through
                else {
					
                }
                break; //}

			case GENERIC_DRIVE_STATE: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
				// *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    // robot.resetDriveEncoders();  // Uncomment if encoder measurement should be reset
					// waitTime = DriveStateTime.milliseconds() + timeToWait; // Replace timeToWait with milliseconds
					// speedDrive(speed);
					// speedStrafe(speed);  // Positive speed is right
					// robot.gyroDrive(speed, heading, gain);   // Recommended gain around 0.03
					// robot.gyroStrafe(speed, heading, gain);  // Positive speed is right, Recommended gain around 0.03
					// turn(leftSpeed, RightSpeed);
                    InitDriveState = false;
				// *** Reasons to leave state ***
                if (DriveStateTime.milliseconds() > waitTime) {  // timeToWait has elapsed
                    stopDrive();
                    newDriveState(DriveState.NEXT_STATE);
				} else if (gotLine()) { // Line Detected	
                    stopDrive();
                    newDriveState(DriveState.NEXT_STATE);
                } else if (gotDistance (targetDistance, true)) { // Drove to distance, TRUE is moving forward
                    stopDrive();
                    newDriveState(DriveState.NEXT_STATE); 
				} else if (gotStrafeDistance(targetDistance, true)){ // Strafed to distance, TRUE is moving right
                    stopDrive();
                    newDriveState(DriveState.NEXT_STATE); 
                } else if (gotAngle(targetHeading,true)) {  // Turned to heading, TRUE is clockwise
                    stopDrive();
                    newDriveState(DriveState.NEXT_STATE); 
				// *** Things to do every time the state is looped through ***
                } else {
					// robot.gyroDrive(speed, heading, gain);  // Recommended gain around 0.03
					// robot.gyroStrafe(speed, heading, gain);  // Positive speed is right, Recommended gain around 0.03
					// checkLowestDSensor();
                }
                break; //}

			case GENERIC_ARM_STATE: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
				// *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    // waitTime = DriveStateTime.milliseconds() + timeToWait; // Replace timeToWait with milliseconds
					// robot.lg_servo.setPosition(servo position);
					// robot.rg_servo.setPosition(servo position);
                    InitDriveState = false;
				// *** Reasons to leave state ***
                if (DriveStateTime.milliseconds() > waitTime) {  // timeToWait has elapsed
                    newDriveState(DriveState.NEXT_STATE);
                } else if (robot.elbowWithinRange(elbowTargetPosition)){ // Elbow move finished
                    newDriveState(DriveState.NEXT_STATE); 
				} else if (gotStrafeDistance(targetDistance, true)){ // Strafed to distance, TRUE is moving right
                    newDriveState(DriveState.NEXT_STATE); 
                } else if (gotAngle(targetHeading,true)) {  // Turned to heading, TRUE is clockwise
                    newDriveState(DriveState.NEXT_STATE); 
				} else if (DriveStateTime.milliseconds() > waitTime) {  // timeToWait has elapsed
                    newDriveState(DriveState.NEXT_STATE);
				// *** Things to do every time the state is looped through ***
                } else {
					// armControl (robot.ELBOW_MAX_SPEED, elbowTargetPosition, wristPlace, .02);  // If wrist does not move
					// armControl (robot.ELBOW_MAX_SPEED, elbowTargetPosition, wristTargetPosition, .02);  // if wrist does move
                }
                break; //}
				
				/*
				**** Example flag fields for compound if statements ***
				
				propLocation == Prop.RIGHT
                propLocation == Prop.LEFT
				propLocation == Prop.CENTER
				propLocation == Prop.UNKNOWN
				colorSetting == Color.RED
				colorSetting == Color.BLUE
				sideSetting == Side.BACKDROP
				sideSetting == Side.AIRSTRIP
				backPixel
				!backPixel
				park
				!park
				
				Example:
				
				
				} else if (((propLocation == Prop.RIGHT) && (gotStrafeDistance(1.5,true)  )) ||
                           ((propLocation == Prop.LEFT)  && (gotStrafeDistance(-1.5,false)))) {
                    speedDrive(0);           
                    newDriveState(DriveState.SLIGHT_DROP_70);
                }
						   
				*/
				
 