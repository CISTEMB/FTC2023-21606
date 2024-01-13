/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import java.util.Map;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware
{
  // Declare devices(motors, sensors, etc.) {
  public DcMotor lf_motor = null;
  public DcMotor rf_motor = null;
  public DcMotor lb_motor = null;
  public DcMotor rb_motor = null;
  public DcMotor elbow_motor = null;
  public DcMotor left_hang_motor = null;
  public DcMotor right_hang_motor = null;
  public Servo wrist_servo = null;
  public Servo lg_servo = null; 
  public Servo rg_servo = null;
  public Servo launch_servo = null;
  public Servo left_hang_servo = null;
  public Servo right_hang_servo = null;
  public ColorSensor clrl_sensor = null;
  public DistanceSensor leftd_sensor = null;
  public DistanceSensor rightd_sensor = null;
  public DistanceSensor centerd_sensor = null;
  public IMU imu = null;
  
  // }
  
  // Declare IMU stuff {
  public YawPitchRollAngles orientation;
  public double lastAngle;
  public double angle;
  long gyroStrafeLastUpdateTime = System.currentTimeMillis();
  long gyroDriveLastUpdateTime = System.currentTimeMillis();
  //double currentAngle = getAngle();
    
  // }

  // Declare internal variables(initialization status, errors, etc.) {
  public boolean initialized = false;
  private OpMode opmode = null;
  // }

  // Declare variables purely for convience {
  private HardwareMap hardwareMap = null;
  private Telemetry telemetry = null;
  // }

  // Declare "constants" that can be changed by the host class {
  public double MAX_MOTOR_POWER = 1.0;
  public double TURN_SENSITIVITY = 0.5;
  public double DRIVE_SENSITIVITY = 0.75;
  public double STRAFE_SENSITIVITY = 0.75;
  public double WRIST_SENSITIVITY = 0.001;
  public double ELBOW_SENSITIVITY = 0.5;
  public double SLOW_FACTOR = 0.2;
  public double GRIPPER_MID = 0.5;

  public double WRIST_TUCK = 0;  // reverse for old robot
  public int ELBOW_TUCK = 0;
  public int ELBOW_PRETUCK = 20;
  public int ELBOW_PREPICKUP = 20;
  public int PRETUCK_RANGE = 10;
  public double WRIST_BACK = 0.4;   
  public int ELBOW_BACK = 508;
  public double WRIST_PICKUP = 1;
  public int ELBOW_PICKUP = 0;
  public int ELBOW_SLIGHT_RAISE = 20;
  public int ELBOW_FRONT_DROP = 97;
  public double WRIST_FRONT_DROP = 0.887657;
  public double ELBOW_MAX_SPEED = .75;
  public double ELBOW_PRETUCK_MAX_SPEED = 1;
  
  public double RIGHT_GRIP_CLOSED = 0.5;  // right grip closes on high
  public double RIGHT_GRIP_OPEN = 0.2;
  public double LEFT_GRIP_CLOSED = 0.38; //left grip closes on low
  public double LEFT_GRIP_OPEN = 0.65; 
  public double LEFT_GRIP_SLIGHT_OPEN = 0.52;
  public double RIGHT_GRIP_SLIGHT_OPEN = 0.35;

  public double LEFT_HANG_SERVO_INIT = 0;
  public double RIGHT_HANG_SERVO_INIT = 1;
  public double LEFT_HANG_SERVO_RELEASE = 1;
  public double RIGHT_HANG_SERVO_RELEASE = 0;
  
  public double CHECK_DELAY = 20;
  // }
  
  // declare constants for autonomous {
  public int RED_LIMIT = 700;
  public int BLUE_LIMIT = 1500;
  // }
  
  // declare constants for launcher
  public double LAUNCH_DRONE = 0;
  public double HOLD_DRONE = 1;
  //}

  public boolean SIDE_SENSOR_FAULT = false;

  /**
   * Create a RobotHardware instance and return it
   * The init() method MUST be called afterward before any other method is called!
   * @param OpMode The OpMode that this instance will use
   */
  public RobotHardware (OpMode OpMode) { // OpMode is the argument, opmode is the global variable
    opmode = OpMode;
  }

  /**
   * Initialize the hardware devices and prepare them for use
   * @return A boolean signifying if the initialization was successful
   */
  public boolean init() {
    if (initialized) {
      opmode.telemetry.addData("RobotHardware","Error: Already initialized!");
      return false;
    }
    try {
      telemetry = opmode.telemetry;
      if (telemetry == null) {
        return false;
      }
      hardwareMap = opmode.hardwareMap;
      if (hardwareMap == null) {
        opmode.telemetry.addData("RobotHardware","Error: hardwareMap has been incorrectly defined! Have you configured your robot?");
        return false;
      }
      lf_motor = hardwareMap.get(DcMotor.class, "LF_MOTOR");
      rf_motor = hardwareMap.get(DcMotor.class, "RF_MOTOR");
      lb_motor = hardwareMap.get(DcMotor.class, "LB_MOTOR");
      rb_motor = hardwareMap.get(DcMotor.class, "RB_MOTOR");
      elbow_motor = hardwareMap.get(DcMotor.class, "ELBOW_MOTOR");   
      left_hang_motor = hardwareMap.get(DcMotor.class, "LEFT_HANG_MOTOR");
      right_hang_motor = hardwareMap.get(DcMotor.class, "RIGHT_HANG_MOTOR");
      wrist_servo = hardwareMap.get(Servo.class, "WRIST_SERVO");
      lg_servo = hardwareMap.get(Servo.class, "LG_SERVO");
      rg_servo = hardwareMap.get(Servo.class, "RG_SERVO");
      launch_servo = hardwareMap.get(Servo.class, "LAUNCH_SERVO");
      left_hang_servo = hardwareMap.get(Servo.class, "LEFT_HANG_SERVO");
      right_hang_servo = hardwareMap.get(Servo.class, "RIGHT_HANG_SERVO");
      clrl_sensor = hardwareMap.get(ColorSensor.class, "CLRL_SENSOR");
      leftd_sensor = hardwareMap.get(DistanceSensor.class, "LEFTD_SENSOR");
      rightd_sensor = hardwareMap.get(DistanceSensor.class, "RIGHTD_SENSOR");
      
      imu = hardwareMap.get(IMU.class, "IMU");
      
      lf_motor.setDirection(DcMotor.Direction.REVERSE);
      rf_motor.setDirection(DcMotor.Direction.FORWARD);
      lb_motor.setDirection(DcMotor.Direction.FORWARD);
      rb_motor.setDirection(DcMotor.Direction.REVERSE);
      elbow_motor.setDirection(DcMotor.Direction.FORWARD);
      left_hang_motor.setDirection(DcMotor.Direction.REVERSE);
      right_hang_motor.setDirection(DcMotor.Direction.REVERSE);
      
      lf_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rf_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      lb_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rb_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      elbow_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      left_hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      right_hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      
      
      lf_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rf_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      lb_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rb_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      left_hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      right_hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      elbow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      elbow_motor.setTargetPosition(0);
      elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      wrist_servo.setPosition(0);
      
      // TOFIX setGripperPosition(LEFT_GRIP_CLOSED, RIGHT_GRIP_CLOSED);
      setLeftGripperPosition(LEFT_GRIP_CLOSED);
      setRightGripperPosition(RIGHT_GRIP_CLOSED);
      
      launch_servo.setPosition(HOLD_DRONE);

      RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
      RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
      RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
      imu.initialize(new IMU.Parameters(orientationOnRobot));
      resetIMU();
      
      left_hang_servo.setPosition(LEFT_HANG_SERVO_INIT);
      right_hang_servo.setPosition(RIGHT_HANG_SERVO_INIT);

      telemetry.addData("RobotHardware","Initialized successfully!");
      initialized = true;
      return true;
    } catch (Exception e) {
      opmode.telemetry.addData("RobotHardware","Error: %s",e.getMessage());
      return false;
    }
  }

  /**
   * Set the power of the drive motors
   * @param lfPower The power of the left front motor
   * @param rfPower The power of the right front motor
   * @param lbPower The power of the left back motor
   * @param rbPower The power of the right back motor
   */
  public void setDrivePower(double lfPower, double rfPower, double lbPower, double rbPower) {
    lf_motor.setPower(lfPower);
    rf_motor.setPower(rfPower);
    lb_motor.setPower(lbPower);
    rb_motor.setPower(rbPower);
    telemetry.addData("RobotHardware","lfPower: (%.2f), rfPower: (%.2f), lbPower: (%.2f), rbPower: (%.2f)",lfPower,rfPower,lbPower,rbPower);
  }

  public void resetDriveEncoders() {
    lf_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rf_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lb_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rb_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
  
  public void setDriveMode(DcMotor.RunMode mode) {
    lf_motor.setMode(mode);
    rf_motor.setMode(mode);
    lb_motor.setMode(mode);
    rb_motor.setMode(mode);
  }
  
  public void setDrivePosition(int countslf, int countsrf, int countslb, int countsrb) {
    lf_motor.setTargetPosition(countslf);
    rf_motor.setTargetPosition(countsrf);
    lb_motor.setTargetPosition(countslb);
    rb_motor.setTargetPosition(countsrb);
  }

  /**
   * Set the power of the elbow motor
   * @param power The power of the elbow motor
   */
  public void setElbowPower(double power) {
    elbow_motor.setPower(power);
    telemetry.addData("RobotHardware","Elbow power: (%.2f)",power);
  }

  /**
   * Set the position of the wrist servo
   * @param position The position of the wrist servo
   */
  public void setWristPosition(double position) {
    wrist_servo.setPosition(position);
    telemetry.addData("RobotHardware","Wrist position: (%.2f)",position);
  }

  /**
   * Set the position of the gripper servos together
   * @param position The position of the grippers
   */
  public void setRightGripperPosition(double pos) {
    rg_servo.setPosition(pos);
    telemetry.addData("RobotHardware","rg position: (%.2f)", pos);
  }
  
  public void setLeftGripperPosition(double pos) {
    lg_servo.setPosition(pos);
    telemetry.addData("RobotHardware","lg position: (%.2f)", pos);
  }

  public void driveRobot(double forwardSpeed, double turnSpeed, double strafeSpeed, boolean slowmode_state) {
    // Calculate power {
    
    double speedfactor;
    if (slowmode_state) {
      speedfactor = SLOW_FACTOR;
    } else {
      speedfactor = 1;
    }
    telemetry.addData("speedfactor: ", speedfactor);
    double leftFrontPower = Range.clip(speedfactor*((TURN_SENSITIVITY*turnSpeed)+(DRIVE_SENSITIVITY*forwardSpeed)+(STRAFE_SENSITIVITY*strafeSpeed)),
                                -MAX_MOTOR_POWER,
                                MAX_MOTOR_POWER
    );
    double rightFrontPower = Range.clip(speedfactor*(-(TURN_SENSITIVITY*turnSpeed)+(DRIVE_SENSITIVITY*forwardSpeed)-(STRAFE_SENSITIVITY*strafeSpeed)),
                                -MAX_MOTOR_POWER,
                                MAX_MOTOR_POWER
    );
    double leftBackPower = Range.clip(speedfactor*((TURN_SENSITIVITY*turnSpeed)+(DRIVE_SENSITIVITY*forwardSpeed)-(STRAFE_SENSITIVITY*strafeSpeed)),
                                -MAX_MOTOR_POWER,
                                MAX_MOTOR_POWER
    );
    double rightBackPower = Range.clip(speedfactor*(-(TURN_SENSITIVITY*turnSpeed)+(DRIVE_SENSITIVITY*forwardSpeed)+(STRAFE_SENSITIVITY*strafeSpeed)),
                                -MAX_MOTOR_POWER,
                                MAX_MOTOR_POWER
    );
    telemetry.addData("motor powers", "lf: (%.2f), rf: (%.2f), lb: (%.2f), rb: (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    // }

    this.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    
  }

  public boolean elbowWithinRange(int target) {
    int elbow = elbow_motor.getCurrentPosition();
    return elbow > target - PRETUCK_RANGE &&
    elbow < target + PRETUCK_RANGE;
  }
  
  public double telHeading = 0;
  public int telElbowPos = 0;
  
  void updatePersistentTelemetry() {
    telemetry.addData("Gyro Heading", telHeading);
    telemetry.addData("Elbow Position", telElbowPos);
  }

  public void resetIMU() {
    imu.resetYaw();
  }
    
  public double getAngle() {
    orientation = imu.getRobotYawPitchRollAngles();
    double currAngle = orientation.getYaw(AngleUnit.DEGREES);
      
    double deltaAngle = currAngle - lastAngle;
      
    if (deltaAngle < -180)
      deltaAngle += 360;
    else if (deltaAngle > 180)
      deltaAngle -= 360;
    
    angle += deltaAngle;
      
    lastAngle = currAngle;
      
    telHeading = angle;
    return angle;
  }
    
    
  public void gyroDrive(double speed, double targetAngle, double gain ) {     
    setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Update gyro readings only if enough time has passed
    if (System.currentTimeMillis() - gyroDriveLastUpdateTime > CHECK_DELAY) {
      angle = getAngle();
      double error = targetAngle - angle;
          
      double leftFrontPower = speed - (gain * error);
      double rightFrontPower = speed + (gain * error);
      double leftBackPower = speed - (gain * error);
      double rightBackPower = speed + (gain * error);
          
      // Limit motor powers to the valid range (-1 to 1)
      leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
      rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
      leftBackPower = Range.clip(leftBackPower, -1.0, 1.0);
      rightBackPower = Range.clip(rightBackPower, -1.0, 1.0);
          
      // Apply motor powers to the motors
      setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
          
      // Update the last update time
      gyroDriveLastUpdateTime = System.currentTimeMillis();
    }
  }
    
  public void gyroStrafe(double speed, double targetAngle, double gain ) {
    setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Update gyro readings only if enough time has passed
    if (System.currentTimeMillis() - gyroStrafeLastUpdateTime > CHECK_DELAY) {
      angle = getAngle();
      double error = targetAngle - angle;
        
      double leftFrontPower = speed - (gain * error);
      double rightFrontPower = -speed + (gain * error);
      double leftBackPower = -speed - (gain * error);
      double rightBackPower = speed + (gain * error);
          
      // Limit motor powers to the valid range (-1 to 1)
      leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
      rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
      leftBackPower = Range.clip(leftBackPower, -1.0, 1.0);
      rightBackPower = Range.clip(rightBackPower, -1.0, 1.0);
          
      // Apply motor powers to the motors
      setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
          
      // Update the last update time
      gyroStrafeLastUpdateTime = System.currentTimeMillis();
    }
  }
    
  public void releaseHangServos() {
    left_hang_servo.setPosition(LEFT_HANG_SERVO_RELEASE);
    right_hang_servo.setPosition(RIGHT_HANG_SERVO_RELEASE);
    telemetry.addData("RobotHardware","Hang servos released!");
  }
    
  public void setHangPower(double power) {
    left_hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right_hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    left_hang_motor.setPower(power);
    right_hang_motor.setPower(power);
    telemetry.addData("RobotHardware","Hang motor power: (%.2f)",power);
  }

  public boolean gotSideDistance(double distance, boolean rightSide, boolean movingToward) {
    double sideDistance = -1;
    if (rightSide) { //Right side
      sideDistance = rightd_sensor.getDistance(DistanceUnit.INCH);
    } else { //Left side
      sideDistance = leftd_sensor.getDistance(DistanceUnit.INCH);
    }
	if (sideDistance > 321) {
		SIDE_SENSOR_FAULT = true;
	}
    telemetry.addData("dist", sideDistance);
    if (movingToward) { //Moving toward the object
      if (sideDistance <= distance) {
        return true;
      } else {
        return false;
      }
    } else { //Moving away from the object
      if (sideDistance >= distance) {
        return true;
      } else {
        return false;
      }
    }
  }

}
