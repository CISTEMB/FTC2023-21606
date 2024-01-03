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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RobotHardware;

/*
 * Add a line that says "@Disabled" line to remove this OpMode from the Driver Station OpMode list
 */

@TeleOp(name="Teleop Code", group="TeleOps")

public class TeleopCode extends OpMode
{
    private RobotHardware robot = new RobotHardware(this);
    
    // Declare controller related variables(prev button presses, etc.) {
    private double turn_joy = 0;
    private double drive_joy = 0;
    private double strafe_joy = 0;
    
    private double elbow_joy = 0;
    private double wrist_joy = 0;
    
    private boolean pickup_pos_btn;                // current btn state
    private boolean pickup_pos_btn_old = false;    // prev btn state
    private boolean pickup_pos_btn_press = false;  // when btn changes from false to true
    private boolean back_pos_btn;
    private boolean back_pos_btn_old = false;
    private boolean back_pos_btn_press = false;
    private boolean tuck_pos_btn;
    private boolean tuck_pos_btn_old = false;
    private boolean tuck_pos_btn_press = false;
    private boolean reset_btn;
    private boolean reset_btn_old = false;
    private boolean reset_btn_press = false;
    
    private boolean right_grip_btn;
    private boolean left_grip_btn;
    
    private boolean slowmode_btn = false;
    private boolean slowmode_btn_old = false;
    private boolean slowmode_state = false;
    
    private boolean d1Launch_btn = false;
    private boolean d2Launch_btn = false;
    // }
    
    // Declare general global variables {
    private ElapsedTime runtime = new ElapsedTime();
    // }

    // State machine enums/variables {
    
    // Declare arm state machine enums and variables {
    private enum ArmState {
        ARM_STATE_INIT,
        ARM_STATE_MANUAL,
        ARM_STATE_PICKUP,
        ARM_STATE_BACK,
        ARM_STATE_HOLD,
        ARM_STATE_PRETUCK,
        ARM_STATE_PREPICKUP,
        ARM_STATE_TUCK,
        ARM_STATE_ELBOW_HOLD,
        ARM_STATE_END, // Might not need, there for structure's sake
    };
    
    private ArmState CurrentArmState;
    private boolean InitArmState = false;
    private ElapsedTime ArmStateTime = new ElapsedTime();
 
    /////Arm Preset Constants ////// {
    private double wristPlace = 0;
    private int elbowHold = 0;
    //}
    
    // }
    
    // Declare drive state machine enums and variables {
    private enum DriveState {
        DRIVE_STATE_INIT,
        DRIVE_STATE_CONTROL,
        DRIVE_STATE_END,      // Might not need, there for structure's sake
    };
    
    private DriveState CurrentDriveState;
    private boolean InitDriveState = false;
    private ElapsedTime DriveStateTime = new ElapsedTime();
    // }
    
    // }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        boolean initializationStatus = robot.init();
        if (!initializationStatus) {
            telemetry.addData("Status", "Initialization failed!");
            throw new RuntimeException("Initialization failed!");
        }
        
        // Initialize states for state machines {
        newDriveState(DriveState.DRIVE_STATE_INIT);
        newArmState(ArmState.ARM_STATE_INIT);
        // }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Read controllers {
        // Gamepad 1
        drive_joy = -gamepad1.left_stick_y;
        turn_joy  =  gamepad1.left_stick_x;
        strafe_joy = gamepad1.right_stick_x;
        telemetry.addData("Drive Joysticks", "drive (%.2f), turn (%.2f), strafe (%.2f)", drive_joy, turn_joy, strafe_joy);
        
        slowmode_btn = gamepad1.left_bumper;
        if (!slowmode_btn_old && slowmode_btn) {
            slowmode_state = !slowmode_state;
        }
        slowmode_btn_old = slowmode_btn;
        telemetry.addData("Slowmode ", "button(lb): " + slowmode_btn + ", state: " + slowmode_state);
        
        elbow_joy = gamepad2.left_stick_y;
        wrist_joy = gamepad2.right_stick_y;
        telemetry.addData("Arm Joysticks", "arm (%.2f), wrist (%.2f)", elbow_joy, wrist_joy);
        
        pickup_pos_btn = gamepad2.a;
        if (!pickup_pos_btn_old && pickup_pos_btn) {
            pickup_pos_btn_press = true;
        } else {
            pickup_pos_btn_press = false;
        }
        pickup_pos_btn_old = pickup_pos_btn;
        reset_btn = gamepad2.left_bumper;
        if (!reset_btn_old && reset_btn) {
            reset_btn_press = true;
        } else {
            reset_btn_press = false;
        }
        reset_btn_old = reset_btn;
        tuck_pos_btn = gamepad2.y;
        if (!tuck_pos_btn_old && tuck_pos_btn) {
            tuck_pos_btn_press = true;
        } else {
            tuck_pos_btn_press = false;
        }
        tuck_pos_btn_old = tuck_pos_btn;
        back_pos_btn = (gamepad2.b && !gamepad2.start);
        if (!back_pos_btn_old && back_pos_btn) {
            back_pos_btn_press = true;
        } else {
            back_pos_btn_press = false;
        }
        back_pos_btn_old = back_pos_btn;
        telemetry.addData("Pos Btns", "pu(y): " + pickup_pos_btn + ", tuck(a): " + tuck_pos_btn + ", back(b): " + back_pos_btn + back_pos_btn_press);
        
        if (gamepad2.right_trigger > .5 || gamepad1.right_trigger > .5) {
            right_grip_btn = true;
        } else {
            right_grip_btn = false;
        }
		if (gamepad2.left_trigger > .5 || gamepad1.left_trigger > .5) {
            left_grip_btn = true;
        } else {
            left_grip_btn = false;
        }
		
        // grip_mid_btn = (gamepad2.right_bumper||gamepad1.right_bumper);
        telemetry.addData("Grip Buttons", "right (rt): " + right_grip_btn + "; left (lt): " + left_grip_btn );
        
        // launcher buttons
        d1Launch_btn = gamepad1.guide;
        d2Launch_btn = gamepad2.guide;
        
        telemetry.addData("Launch Buttons", "D1(center): " + d1Launch_btn + ", D2(Center): " + d2Launch_btn );
         
        // }
        
        switch (CurrentArmState) {
            case ARM_STATE_INIT://{
                telemetry.addData("Arm state", "Init");
                if (InitArmState) {
                    robot.elbow_motor.setPower(0);
                    
                    InitArmState = false;
                }
                if (true) {
                    newArmState(ArmState.ARM_STATE_TUCK);
                }
                break;//}
            case ARM_STATE_MANUAL://{
                telemetry.addData("Arm state", "Manual");
                if (InitArmState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // TODO: Check if RUN_USING_ENCODER is correct or if we should use RUN_USING_ENCODERS
                    InitArmState = false;
                } 
                if (tuck_pos_btn) {
                    newArmState(ArmState.ARM_STATE_PRETUCK);
                } else if (pickup_pos_btn_press) {
                    newArmState(ArmState.ARM_STATE_PREPICKUP);
                } else if (-0.01<elbow_joy && elbow_joy<0.01){
                    elbowHold = robot.elbow_motor.getCurrentPosition();
                    newArmState(ArmState.ARM_STATE_ELBOW_HOLD);
                } else if (reset_btn_press) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    InitArmState = true; // Probably bad practice, but oh well
                }else {
                    robot.setElbowPower(elbow_joy * robot.ELBOW_SENSITIVITY);
                    wristPlace -= wrist_joy * robot.WRIST_SENSITIVITY;
                    wristPlace = Range.clip(wristPlace, 0, 1);
                    robot.setWristPosition(wristPlace);
                    
                    gripperControl(right_grip_btn, left_grip_btn);
                }
                break;//}
            case ARM_STATE_ELBOW_HOLD://{
                telemetry.addData("Arm state", "Elbow Hold");
                if (InitArmState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitArmState = false;
                }
                if (tuck_pos_btn) {
                    newArmState(ArmState.ARM_STATE_TUCK);
                } else if (!(-0.01<elbow_joy && elbow_joy<0.01)) {
                    newArmState(ArmState.ARM_STATE_MANUAL);
                }   else if(back_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_BACK);
                } else if(pickup_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_PREPICKUP);
                } else if(tuck_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_PRETUCK);
                } else {
                    robot.elbow_motor.setTargetPosition(elbowHold);
                    wristPlace -= wrist_joy * robot.WRIST_SENSITIVITY;
                    wristPlace = Range.clip(wristPlace, 0, 1);
                    robot.setWristPosition(wristPlace);
                    telemetry.addData("Wrist location", wristPlace);
                    telemetry.addData("Elbow Location", robot.elbow_motor.getCurrentPosition());
                    
                    gripperControl(right_grip_btn, left_grip_btn);
                }
                break;//}
            
            case ARM_STATE_PRETUCK:
                telemetry.addData("Arm state", "PreTuck");
                if (InitArmState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitArmState = false;
                }
                if (!(-0.01<elbow_joy && elbow_joy<0.01)) {
                    newArmState(ArmState.ARM_STATE_MANUAL);
                }  else if(back_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_BACK);
                } else if(pickup_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_PREPICKUP);
                } else if (robot.elbowWithinRange(robot.ELBOW_PRETUCK)){
                    newArmState(ArmState.ARM_STATE_TUCK);
                } else {
                    armControl (robot.ELBOW_PRETUCK_MAX_SPEED, robot.ELBOW_PRETUCK,  wristPlace , 0);
                    // gripperControl(right_grip_btn, left_grip_btn);
                }
                break;//}
                
            case ARM_STATE_PREPICKUP:
                telemetry.addData("Arm state", "PrePickup");
                if (InitArmState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitArmState = false;
                }
                if (!(-0.01<elbow_joy && elbow_joy<0.01)) {
                    newArmState(ArmState.ARM_STATE_MANUAL);
                }  else if(back_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_BACK);
                } else if(tuck_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_PRETUCK);
                } else if (robot.elbowWithinRange(robot.ELBOW_PREPICKUP)){
                    newArmState(ArmState.ARM_STATE_PICKUP);
                } else {
                    armControl (robot.ELBOW_PRETUCK_MAX_SPEED, robot.ELBOW_PREPICKUP, robot.WRIST_PICKUP, 0.04);
                    // gripperControl(right_grip_btn, left_grip_btn);
                }
                break;//}
                
            case ARM_STATE_TUCK://{
                telemetry.addData("Arm state", "Tuck");
                if (InitArmState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitArmState = false;
                }
                if (!(-0.01<elbow_joy && elbow_joy<0.01)) {
                    newArmState(ArmState.ARM_STATE_MANUAL);
                }  else if(back_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_BACK);
                } else if(pickup_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_PREPICKUP);
                }
                else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_TUCK,  robot.WRIST_TUCK , .02);
                    // gripperControl(right_grip_btn, left_grip_btn);
                }
                break;//}
            
            case ARM_STATE_BACK://{
                telemetry.addData("Arm state", "Back");
                if (InitArmState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitArmState = false;
                }
                if (!(-0.01<elbow_joy && elbow_joy<0.01)) {
                    newArmState(ArmState.ARM_STATE_MANUAL);
                } else if(tuck_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_PRETUCK);
                } else if(pickup_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_PREPICKUP);
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_BACK, robot.WRIST_BACK, .004);
                    gripperControl(right_grip_btn, left_grip_btn);
                }
                break;//}
            
            case ARM_STATE_PICKUP://{
                telemetry.addData("Arm state", "Pickup");
                if (InitArmState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitArmState = false;
                }
                if (!(-0.01<elbow_joy && elbow_joy<0.01)) {
                    newArmState(ArmState.ARM_STATE_MANUAL);
                } else if(tuck_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_PRETUCK);
                } else if(back_pos_btn_press){
                    newArmState(ArmState.ARM_STATE_BACK);
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_PICKUP, robot.WRIST_PICKUP, .1);
                    gripperControl(right_grip_btn, left_grip_btn);
                }
                break;//}
         }
        
        switch (CurrentDriveState){
            case DRIVE_STATE_INIT:
                telemetry.addData("Drive state", "Init");
                if (InitDriveState)
                {
                    robot.setDrivePower(0, 0, 0, 0);
                    
                    InitDriveState = false;
                } 
                if (true) { // Because we not want to immedatly start
                    newDriveState(DriveState.DRIVE_STATE_CONTROL);
                }
                break;
                
            case DRIVE_STATE_CONTROL:
                telemetry.addData("Drive state", "Controlling");
                if (InitDriveState)
                {
                    InitDriveState = false;
                } else {
                    robot.driveRobot(drive_joy, turn_joy, strafe_joy, slowmode_state);
                }
                break;
        }
        
        
        // this is the launch code
        if (d1Launch_btn   && d2Launch_btn   ) {
  //           && run time>= 90
             
            telemetry.addData("Launching", "!!!");
            robot.launch_servo.setPosition(robot.LAUNCH_DRONE);
        }  else {
            robot.launch_servo.setPosition(robot.HOLD_DRONE);
        }
            
        
        

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void newDriveState(DriveState newState) {
        DriveStateTime.reset();
        CurrentDriveState = newState;
        InitDriveState = true;
    }
    
    private void newArmState(ArmState newState) {
        ArmStateTime.reset();
        CurrentArmState = newState;
        InitArmState = true;
    }
    
    private void gripperControl(boolean right_btn, boolean left_btn) {
        if(right_btn) {
			robot.setRightGripperPosition(robot.RIGHT_GRIP_OPEN);
		} else {
			robot.setRightGripperPosition(robot.RIGHT_GRIP_CLOSED);
		}
		if(left_btn) {
			robot.setLeftGripperPosition(robot.LEFT_GRIP_OPEN);
		} else {
			robot.setLeftGripperPosition(robot.LEFT_GRIP_CLOSED);
		}
		telemetry.addData("Gripper", "lg: " + left_btn + ", rg: " + right_btn);
		/* old gripper code
		if (mid_btn) {
            robot.setGripperPosition(robot.LEFT_GRIP_DROP1,robot.RIGHT_GRIP_DROffP1);
            //robot.setGripperPosition(.75, .25);
            //robot.setGripperPosition(1,1);
            telemetry.addData ("GC", "mid");
        } else if (wide_btn) {
            robot.setGripperPosition(robot.LEFT_GRIP_OPEN, robot.RIGHT_GRIP_OPEN);
             telemetry.addData ("GC" ," wide");
        } else {
            robot.setGripperPosition(robot.LEFT_GRIP_CLOSED, robot.RIGHT_GRIP_CLOSED);
             telemetry.addData ("GC" ," clodes");
        } */
    }
    
    private void armControl (double power, int elbow, double wrist, double wristStep) {
                    
        wristPlace=robot.wrist_servo.getPosition();
                   
        if  (-wristStep < wrist - wristPlace && wrist - wristPlace < wristStep) {
            robot.setWristPosition(wrist);
        } else if (wristPlace < wrist) {
            robot.setWristPosition(wristPlace + wristStep);
        } else  {
            robot.setWristPosition(wristPlace - wristStep);
        }
        robot.setElbowPower(power);
        robot.elbow_motor.setTargetPosition(elbow);
                    
        telemetry.addData("Target Wrist location", wrist);
        telemetry.addData("Elbow Location", robot.elbow_motor.getCurrentPosition());
    }
}
