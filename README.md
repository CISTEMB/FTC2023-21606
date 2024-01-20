# FTC2023-21606

This is the code for the Method 2R Madness robot for the 2023-24 Centerstage season

## Code Description

**ACustom.java**Generic Autonomous OpMode setup for setting autonomous starting parameters.

**ACustomHigh.java**Autonomous OpMode setup to drop pixels on backboard from a higher height.

**RobotHardware.java**Hardware abstraction class.

**TeleopCode.java**Teleop OpMode.

**TestSensor.java**OpMode that reports some sensor information.

## To reset robot for play:

### Drone Setup
- Connect drone launch rubber band around relese servo (at end of drone launch guide).
- Drag rubber band through guide, taking care not to twist.
- Loop rubber band around ears on each side of guide and around bottom of aluminum structe of guide.
- Place drone in guide back end first.

### Climber setup
- Release arms by turning arm servo locks inward.  Arms should spring up.
- Make sure hooks are magnetically attached to climbing arms.
- The string should not be taut, and each side should have equal ammounts of string.
- Wind or unwind string to make lengths equal, and not too loose.  
- Fold arms down, and lock into place by turning servo locks.
- Pull string slightly tight between winding wheels and hole with robber band,
- Pull small loop of string under rubber band, keeping string slightly tight between hole and winding wheel.
- Tuck remaining string in square cups.


## To Operate Autonomous:

- Place robot in center of strting square with back placed against side wall, gripper facing awat from the side wall.
- Place Yellow and Purple pixels in appropriate locations
- Tip gripper back into robot.
- Select autonomous opmode asnd press init.
- After init starts, use buttons on controller 2 to set Autonomous settings, using the letter key on the screen.
    - Red or blue indicates color side of field where robot is starting
	- Backboard or Airstrip indicates side of truss the robot is starting from
	- Place pixel yes or no indicates if pixel should be placed on backboard.
	- Park indicates if robot should park after placing pixel.
        - Park has no effect if staring on the airstrip side and placing a pixel.
- Press start to begin autonomous

## To Operate in Teleop

### Controller 1 - Driver

- left stick, drive & turn
- right stick, strafe (side to side)
- Left bumper, slow mode toggle
- left trigger, open left gripper
- right trigger, open right gripper
- logitech button, fire drone (controller 2 must press as well)
- back button, release hanger (controller 2 must press as well)

### Controller 2 - Arm Operator

- y = tuck position (tucks arm inside of robot)
- b = back position (goes to wards high back of bot to put pixels in higher positions on backbard,)
- a = pickup postion (use this to pickup pixels)
- x = front position (use this to place pixels on the backborad in lower positions.)
- left stick, manual move wrist
- right stick, manual mov arm
- left trigger, open left gripper
- right trigger, open right gripper
- back button, release hanger (controller one must press as well)
- logitech button, fire drone (controller one must press as well)

		
		
