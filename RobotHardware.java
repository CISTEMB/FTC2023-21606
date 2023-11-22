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

public class RobotHardware
{
  // Declare devices(motors, sensors, etc.) {
  public DcMotor lf_motor = null;
  public DcMotor rf_motor = null;
  public DcMotor lb_motor = null;
  public DcMotor rb_motor = null;
  public DcMotor elbow_motor = null;
  public Servo wrist_servo = null;
  public Servo lg_servo = null; 
  public Servo rg_servo = null;
  public Servo launch_servo = null;
  public ColorSensor clrl_sensor = null;
  public DistanceSensor leftd_sensor = null;
  public DistanceSensor rightd_sensor = null;
  public DistanceSensor centerd_sensor = null;
  
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
  public int ELBOW_PRETUCK = 150;
  public int ELBOW_PREPICKUP = 180;
  public int PRETUCK_RANGE = 10;
  public double WRIST_BACK = 0.22;   
  public int ELBOW_BACK = 512;
  public double WRIST_PICKUP = 0.8;
  public int ELBOW_PICKUP = 30;
  public double ELBOW_MAX_SPEED = .75;
  
  public double RIGHT_GRIP_CLOSED = 0.12;  // right grip closes on low
  public double RIGHT_GRIP_DROP1 = 0.175;
  public double RIGHT_GRIP_OPEN = 0.25;
  public double LEFT_GRIP_CLOSED = 0.75;  // left grip closes on high
  public double LEFT_GRIP_DROP1 = 0.685;
  public double LEFT_GRIP_OPEN = 0.6;
  // }
  
  // declare constants for autonomous {
  public int RED_LIMIT = 700;
  public int BLUE_LIMIT = 1500;
  // }
  
  // declare constants for launcher
  public double LAUNCH_DRONE = 0;
  public double HOLD_DRONE = 1;
  //}

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
      wrist_servo = hardwareMap.get(Servo.class, "WRIST_SERVO");
      lg_servo = hardwareMap.get(Servo.class, "LG_SERVO");
      rg_servo = hardwareMap.get(Servo.class, "RG_SERVO");
      launch_servo = hardwareMap.get(Servo.class, "LAUNCH_SERVO");
      clrl_sensor = hardwareMap.get(ColorSensor.class, "CLRL_SENSOR");
      leftd_sensor = hardwareMap.get(DistanceSensor.class, "LEFTD_SENSOR");
      rightd_sensor = hardwareMap.get(DistanceSensor.class, "RIGHTD_SENSOR");
      centerd_sensor = hardwareMap.get(DistanceSensor.class, "CENTERD_SENSOR");

      lf_motor.setDirection(DcMotor.Direction.REVERSE);
      rf_motor.setDirection(DcMotor.Direction.FORWARD);
      /*lb_motor.setDirection(DcMotor.Direction.REVERSE);
      rb_motor.setDirection(DcMotor.Direction.FORWARD);*/ // Uncomment these for testing on old robot
      lb_motor.setDirection(DcMotor.Direction.FORWARD);
      rb_motor.setDirection(DcMotor.Direction.REVERSE);
      elbow_motor.setDirection(DcMotor.Direction.FORWARD);
      
      lf_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rf_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      lb_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rb_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      elbow_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      
      
      lf_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rf_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      lb_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rb_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      elbow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      elbow_motor.setTargetPosition(0);
      elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      wrist_servo.setPosition(0);
      setGripperPosition(LEFT_GRIP_CLOSED, RIGHT_GRIP_CLOSED);
      
      launch_servo.setPosition(HOLD_DRONE);

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
  public void setGripperPosition(double left_pos, double right_pos) {
    lg_servo.setPosition(left_pos);
    rg_servo.setPosition(right_pos);
    telemetry.addData("RobotHardware","lg position: (%.2f); rg position: (%.2f) ",  left_pos, right_pos);
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
}
