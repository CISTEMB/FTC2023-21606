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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math.*;

/*
 * Add a line that says "@Disabled" line to remove this OpMode from the Driver Station OpMode list
 */

@TeleOp(name="Teleop Code", group="Practice")

public class TeleopCode extends OpMode
{
    // Declare devices(motors, sensors, etc.) {
    private DcMotor lf_motor = null;
    private DcMotor rf_motor = null;
    private DcMotor lb_motor = null;
    private DcMotor rb_motor = null;
    private DcMotor elbow_motor = null;
    private Servo wrist_servo = null;
    private Servo lg_servo = null; 
    private Servo rg_servo = null;
    
    // }
    
    // Declare controller related variables(prev button presses, etc.) {
    private double turn_joy = 0;
    private double drive_joy = 0;
    
    private double strafe_joy = 0;
    
    private double elbow_joy = 0;
    private double wrist_joy = 0;
    
    private boolean pickup_pos_btn;
    private boolean back_pos_btn;
    private boolean tuck_pos_btn;
    
    private boolean grip_btn;
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
        ARM_STATE_HOLD,
        ARM_STATE_ELBOW_HOLD,
        ARM_STATE_END, // Might not need, there for structure's sake
    };
    
    private ArmState CurrentArmState;
    private boolean InitArmState = false;
    private ElapsedTime ArmStateTime = new ElapsedTime();
    
    private double wristPlace = 0.525;
    private int elbowHold = 0;
    
    private static double WRIST_SENSITIVITY = 0.001;
    private static double ELBOW_SENSITIVITY = 0.5;
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
     
    private static double TURN_SENSITIVITY = 0.5;
    private static double DRIVE_SENSITIVITY = 0.75;
    private static double STRAFE_SENSITIVITY = 0.75;
    private static double MAX_MOTOR_POWER = 1;
    
    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    // }
    
    // }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the driver station).
        lf_motor = hardwareMap.get(DcMotor.class, "LF_MOTOR");
        rf_motor = hardwareMap.get(DcMotor.class, "RF_MOTOR");
        lb_motor = hardwareMap.get(DcMotor.class, "LB_MOTOR");
        rb_motor = hardwareMap.get(DcMotor.class, "RB_MOTOR");
        elbow_motor = hardwareMap.get(DcMotor.class, "ELBOW_MOTOR");   
        wrist_servo = hardwareMap.get(Servo.class, "WRIST_SERVO");
        lg_servo = hardwareMap.get(Servo.class, "LG_SERVO");
        rg_servo = hardwareMap.get(Servo.class, "RG_SERVO");
        
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lf_motor.setDirection(DcMotor.Direction.REVERSE);
        rf_motor.setDirection(DcMotor.Direction.FORWARD);
        lb_motor.setDirection(DcMotor.Direction.FORWARD);
        rb_motor.setDirection(DcMotor.Direction.REVERSE);
        elbow_motor.setDirection(DcMotor.Direction.REVERSE);
        
        lf_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        lf_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow_motor.setTargetPosition(0);
        elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        
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
        drive_joy = -gamepad1.left_stick_y;
        turn_joy  =  gamepad1.left_stick_x;
        strafe_joy = gamepad1.right_stick_x;
        telemetry.addData("Drive Joysticks", "drive (%.2f), turn (%.2f), strafe (%.2f)", drive_joy, turn_joy, strafe_joy);
        
        elbow_joy = gamepad2.left_stick_y;
        wrist_joy = gamepad2.right_stick_y;
        telemetry.addData("Arm Joysticks", "arm (%.2f), wrist (%.2f)", elbow_joy, wrist_joy);
        
        pickup_pos_btn = gamepad2.y;
        tuck_pos_btn = gamepad2.a;
        back_pos_btn = gamepad2.b;
        telemetry.addData("Pos Btns", "pu(y): " + pickup_pos_btn + ", tuck(a): " + tuck_pos_btn + ", back(b): " + back_pos_btn);
        
        if (gamepad2.right_trigger > .5 || gamepad1.right_trigger > .5) {
            grip_btn = true;
        } else {
            grip_btn = false;
        }
        telemetry.addData("Grip  Button", grip_btn);
        
        // }
        
        switch (CurrentArmState) {
            case ARM_STATE_INIT://{
                telemetry.addData("Arm state", "Init");
                if (InitArmState) {
                    elbow_motor.setPower(0);
                    
                    InitArmState = false;
                }
                if (true) {
                    newArmState(ArmState.ARM_STATE_PICKUP);
                }
                break;//}
            case ARM_STATE_PICKUP://{
                telemetry.addData("Arm state", "Pickup");
                if (InitArmState) {
                    InitArmState = false; // Yes this has to be here because otherwise we'll overwrite the new value
                    newArmState(ArmState.ARM_STATE_MANUAL); // Remove this when we have a actual pickup state
                }
                break;//}
            case ARM_STATE_MANUAL:
                telemetry.addData("Arm state", "Manual");
                if (InitArmState) {
                    elbow_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // TODO: Check if RUN_USING_ENCODER is correct or if we should use RUN_USING_ENCODERS
                    InitArmState = false;
                } else if (-0.01<elbow_joy && elbow_joy<0.01){
                    elbowHold = elbow_motor.getCurrentPosition();
                    newArmState(ArmState.ARM_STATE_HOLD);
                } else {
                    elbow_motor.setPower(elbow_joy * ELBOW_SENSITIVITY);
                    wristPlace += wrist_joy * WRIST_SENSITIVITY;
                    wristPlace = Range.clip(wristPlace, 0, 1);
                    wrist_servo.setPosition(wristPlace);
                    telemetry.addData("Wrist location", wristPlace);
                    
                    gripperControl(grip_btn);
                }
                break;
            case ARM_STATE_ELBOW_HOLD://{
                telemetry.addData("Arm state", "Elbow Hold");
                if (InitArmState) {
                    elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitArmState = false;
                } else if (!(-0.01<elbow_joy && elbow_joy<0.01)) {
                    newArmState(ArmState.ARM_STATE_MANUAL);
                } else {
                    elbow_motor.setTargetPosition(elbowHold);
                    wristPlace += wrist_joy * WRIST_SENSITIVITY;
                    wristPlace = Range.clip(wristPlace, 0, 1);
                    wrist_servo.setPosition(wristPlace);
                    telemetry.addData("Wrist location", wristPlace);
                    
                    gripperControl(grip_btn);
                }
                break;//}
        }
        
        switch (CurrentDriveState){
            case DRIVE_STATE_INIT:
                telemetry.addData("Drive state", "Init");
                if (InitDriveState)
                {
                    lf_motor.setPower(0);
                    rf_motor.setPower(0);
                    lb_motor.setPower(0);
                    rb_motor.setPower(0);
                    
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
                    // Calculate power {
                    leftFrontPower = Range.clip((TURN_SENSITIVITY*turn_joy)+(DRIVE_SENSITIVITY*drive_joy)+(STRAFE_SENSITIVITY*strafe_joy),
                                                -MAX_MOTOR_POWER,
                                                MAX_MOTOR_POWER
                    );
                    rightFrontPower = Range.clip(-(TURN_SENSITIVITY*turn_joy)+(DRIVE_SENSITIVITY*drive_joy)-(STRAFE_SENSITIVITY*strafe_joy),
                                                -MAX_MOTOR_POWER,
                                                MAX_MOTOR_POWER
                    );
                    leftBackPower = Range.clip((TURN_SENSITIVITY*turn_joy)+(DRIVE_SENSITIVITY*drive_joy)-(STRAFE_SENSITIVITY*strafe_joy),
                                                -MAX_MOTOR_POWER,
                                                MAX_MOTOR_POWER
                    );
                    rightBackPower = Range.clip(-(TURN_SENSITIVITY*turn_joy)+(DRIVE_SENSITIVITY*drive_joy)+(STRAFE_SENSITIVITY*strafe_joy),
                                                -MAX_MOTOR_POWER,
                                                MAX_MOTOR_POWER
                    );
                    // }
                    
                    // Send calculated power to wheels {
                    lf_motor.setPower(leftFrontPower);
                    rf_motor.setPower(rightFrontPower);
                    lb_motor.setPower(leftBackPower);
                    rb_motor.setPower(rightBackPower);
                    // }
                    
                    telemetry.addData("Motors", "lf (%.2f), rf (%.2f), lb (%.2f), rb (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
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
    
    private void gripperControl(boolean btn) {
        if(btn) {
            rg_servo.setPosition(1);
            lg_servo.setPosition(1);
        } else {
            rg_servo.setPosition(0);
            lg_servo.setPosition(0);
        }
    }
}
