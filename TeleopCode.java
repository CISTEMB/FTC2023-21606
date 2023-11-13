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

@TeleOp(name="Teleop Code", group="Practice")

public class TeleopCode extends OpMode
{
    private RobotHardware robot = new RobotHardware(this);
    
    // Declare controller related variables(prev button presses, etc.) {
    private double turn_joy = 0;
    private double drive_joy = 0;
    
    private double strafe_joy = 0;
    
    private double elbow_joy = 0;
    private double wrist_joy = 0;
    
    private boolean pickup_pos_btn;
    private boolean back_pos_btn;
    private boolean tuck_pos_btn;
    
    private boolean grip_wide_btn;
    private boolean grip_mid_btn;
    private boolean slowmode_btn = false;
    private boolean slowmode_btn_old = false;
    private boolean slowmode_state = false;
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
        ARM_STATE_TUCK,
        ARM_STATE_ELBOW_HOLD,
        ARM_STATE_END, // Might not need, there for structure's sake
    };
    
    private ArmState CurrentArmState;
    private boolean InitArmState = false;
    private ElapsedTime ArmStateTime = new ElapsedTime();
    
    private double wristPlace = 0;
    private int elbowHold = 0;
    private static double WRIST_TUCK = 0;  // reverse for old robot
    private static int ELBOW_TUCK = 0;
    private static int ELBOW_BACK = 546;
    private static double WRIST_BACK = 0.274;
    
    // preset arm pos
    private static double WRIST_PICKUP = 1.0;
    private static int ELBOW_PICKUP = 54;
    
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
        
        pickup_pos_btn = gamepad2.y;
        tuck_pos_btn = gamepad2.a;
        back_pos_btn = gamepad2.b;
        telemetry.addData("Pos Btns", "pu(y): " + pickup_pos_btn + ", tuck(a): " + tuck_pos_btn + ", back(b): " + back_pos_btn);
        
        if (gamepad2.right_trigger > .5 || gamepad1.right_trigger > .5) {
            grip_wide_btn = true;
        } else {
            grip_wide_btn = false;
        }
        grip_mid_btn = (gamepad2.right_bumper||gamepad1.right_bumper);
        telemetry.addData("Grip Buttons", "wide(rt): " + grip_wide_btn + ", mid(rb): " + grip_mid_btn );
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
                    newArmState(ArmState.ARM_STATE_TUCK);
                } else if (-0.01<elbow_joy && elbow_joy<0.01){
                    elbowHold = robot.elbow_motor.getCurrentPosition();
                    newArmState(ArmState.ARM_STATE_ELBOW_HOLD);
                } else {
                    robot.setElbowPower(elbow_joy * robot.ELBOW_SENSITIVITY);
                    wristPlace -= wrist_joy * robot.WRIST_SENSITIVITY;
                    wristPlace = Range.clip(wristPlace, 0, 1);
                    robot.setWristPosition(wristPlace);
                    
                    gripperControl(grip_wide_btn, grip_mid_btn);
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
                }   else if(back_pos_btn){
                    newArmState(ArmState.ARM_STATE_BACK);
                } else if(pickup_pos_btn){
                    newArmState(ArmState.ARM_STATE_PICKUP);
                } else if(tuck_pos_btn){
                    newArmState(ArmState.ARM_STATE_TUCK);
                } else {
                    robot.elbow_motor.setTargetPosition(elbowHold);
                    wristPlace -= wrist_joy * robot.WRIST_SENSITIVITY;
                    wristPlace = Range.clip(wristPlace, 0, 1);
                    robot.setWristPosition(wristPlace);
                    telemetry.addData("Wrist location", wristPlace);
                    telemetry.addData("Elbow Location", robot.elbow_motor.getCurrentPosition());
                    
                    gripperControl(grip_wide_btn, grip_mid_btn);
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
                }  else if(back_pos_btn){
                    newArmState(ArmState.ARM_STATE_BACK);
                } else if(pickup_pos_btn){
                    newArmState(ArmState.ARM_STATE_PICKUP);
                }
                else {
                    armControl (.25, ELBOW_TUCK,  WRIST_TUCK , .004);
                    gripperControl(grip_wide_btn, grip_mid_btn);
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
                } else if(tuck_pos_btn){
                    newArmState(ArmState.ARM_STATE_TUCK);
                } else if(pickup_pos_btn){
                    newArmState(ArmState.ARM_STATE_PICKUP);
                } else {
                    armControl (.25, ELBOW_BACK, WRIST_BACK, .004);
                    gripperControl(grip_wide_btn, grip_mid_btn);
                }
                break;//}
            
            case ARM_STATE_PICKUP://{
                telemetry.addData("Arm state", "Back");
                if (InitArmState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitArmState = false;
                }
                if (!(-0.01<elbow_joy && elbow_joy<0.01)) {
                    newArmState(ArmState.ARM_STATE_MANUAL);
                } else if(tuck_pos_btn){
                    newArmState(ArmState.ARM_STATE_TUCK);
                } else if(back_pos_btn){
                    newArmState(ArmState.ARM_STATE_BACK);
                } else {
                    armControl (.25, ELBOW_PICKUP, WRIST_PICKUP, .004);
                    gripperControl(grip_wide_btn, grip_mid_btn);
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
    
    private void gripperControl(boolean wide_btn, boolean mid_btn) {
        if (mid_btn) {
            robot.setGripperPosition(1-robot.GRIPPER_MID, robot.GRIPPER_MID);
            //robot.setGripperPosition(.75, .25);
            //robot.setGripperPosition(1,1);
            telemetry.addData ("GC", "mid");
        } else if (wide_btn) {
            robot.setGripperPosition(0, 1);
             telemetry.addData ("GC" ," wide");
        } else {
            robot.setGripperPosition(1, 0);
             telemetry.addData ("GC" ," clodes");
        }
    }
    
    private void armControl (double power, int elbow, double wrist, double wristStep) {
          robot.setElbowPower(power);
        robot.elbow_motor.setTargetPosition(elbow);
                    
        wristPlace=robot.wrist_servo.getPosition();
                   
        if  (-wristStep < wrist - wristPlace && wrist - wristPlace < wristStep) {
            robot.setWristPosition(wrist);
        } else if (wristPlace < wrist) {
            robot.setWristPosition(wristPlace + wristStep);
        } else  {
            robot.setWristPosition(wristPlace - wristStep);
        }
                    
        telemetry.addData("Target Wrist location", wrist);
        telemetry.addData("Elbow Location", robot.elbow_motor.getCurrentPosition());
    }
}
