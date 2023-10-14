/*
Copyright 2022 FIRST Tech Challenge Team 21606

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Disabled
@TeleOp(name="Game: TeleOp Drive", group="Game")

public class GameDrive extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_1;
    private DcMotor Lb_Motor;
    private DcMotor Lf_Motor;
    private DcMotor Rb_Motor;
    private DcMotor Rf_Motor;
    private DcMotor Arm_Motor;
    private Servo Wrist_Servo;
    private Servo Lg_Servo;
    private Servo Rg_Servo;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        Lb_Motor = hardwareMap.get(DcMotor.class, "Lb-motor");
        Lf_Motor = hardwareMap.get(DcMotor.class, "Lf-motor");
        Rb_Motor = hardwareMap.get(DcMotor.class, "Rb-motor");
        Rf_Motor = hardwareMap.get(DcMotor.class, "Rf-motor");
        Arm_Motor = hardwareMap.get(DcMotor.class, "Arm-motor");
        Wrist_Servo = hardwareMap.get(Servo.class, "Wrist_Servo");
        Lg_Servo = hardwareMap.get(Servo.class, "Lg_Servo");
        Rg_Servo = hardwareMap.get(Servo.class, "Rg_Servo");
        
        // ****** Drive Motor Setup ****** //
        //Setting up the variables for drive
        double pc=0.5;  //power coeficcient
        double slowSpeed=0.2; //The slow speed limit
        double fastSpeed=0.65; //The fast speed limit
        double sl=fastSpeed; //speed limit
        double x=0;     //joystick x position
        double y=0;     //joystick y position
        double s=0;     //joystick 2 x position
        boolean slowMode = false; //Is it in slow mode?
        boolean lastX = false; //The last state of x
        boolean newX = false;
        
        //Set variables for motor speed
        double lfc=0;   //Left front motor speed
        double rfc=0;   //Right front motor speed
        double lbc=0;   //Left back motor speed
        double rbc=0;   //Right back motor speed
        
        //Set motors to correct direction
        Lf_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Lb_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Rf_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Rb_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ****** Arm Motor Setup ******//
        // Setup variables for arm 
        //boolean D_pad_up;
        //boolean D_pad_down;
        double armStick = 0;
        int armPresetPos = 1;
        boolean lastDPadState = false;
        boolean reset_X = false;
        boolean reset_A = false;
        int lastArmPosition = Arm_Motor.getCurrentPosition();
        int currentArmPosition;
        int armResetLimit = 57;
        int armPositionToHighPower = 505;
        boolean tuck_Y = false;
        boolean untuck_B = false;
        boolean lastTuck_Y = false;
        boolean lastUntuck_B = false;
        
        int armPlace = 0;
        int armAdd = 1;
        int armMax = 470;
        double armUpPower = 0.5;
        double armDownPower = -0.15;
        double armHoldPower = 0.25;
        boolean armInPowerMode = false;
        
        //Setting up the arm motors
        Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Motor.setTargetPosition(armPlace);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        // ****** Gripper setup ****** //
        // Set-Up Variables for Gripper
        double wristPlace = 0.525;   //The postition the wrist is being held at
        double wristAdd = 0.001; //The maximum value that the wrist servo can be changed by
        double wristStick;       //The reading of the stick used to control the  wrist servo
        double futureWristTime = 0; //The time when the future wrist position is givet to the wrist
        boolean futureWristReady = false; //tells that there is a future commant to wrist
        double futureWristDelay = 0.5; // How long the wait is for wrist to move
        double futureSmallWristDelay = 0.1;
        double futureBigWristDelay = 1.5;
        double futureWristPlace = 1; //Position of wrist for future
        double clawPosition;     //The position of the claw servos
        double clawButton1;       //The button from gamepad1 to activate the claw/gripper
        double clawButton2;       //The button from gamepad2 to activate the claw/gripper
        
        //Set-Up Gripper
        Wrist_Servo.setPosition(wristPlace);
        
        // ****** Set-Up Telemtry ****** //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            
            // ****** Drive Motor Control ****** //
            //Get gamepad values
            y = gamepad1.left_stick_y;  //Left stick y
            x = gamepad1.left_stick_x;  //Left stick x
            s = gamepad1.right_stick_x; //Right stick x
            newX = gamepad1.x;
            if (newX && !lastX) {
                slowMode = !slowMode;
                if (slowMode) {
                    sl = slowSpeed;
                } else {
                    sl = fastSpeed;
                }
            }
            lastX = newX;
            
            
            //Calculate values
            lfc = Range.clip((y-x-s)*pc,-sl,sl);
            rfc = Range.clip((y+x+s)*pc,-sl,sl);
            lbc = Range.clip((y-x+s)*pc,-sl,sl);
            rbc = Range.clip((y+x-s)*pc,-sl,sl);
            
            //Set motor power
            Lf_Motor.setPower(lfc);
            Rf_Motor.setPower(rfc);
            Lb_Motor.setPower(lbc);
            Rb_Motor.setPower(rbc);
            
            //Motor Telemetry
            telemetry.addData("Slow mode",slowMode);
            telemetry.addData("Lf_Motor", lfc);
            telemetry.addData("Rf_Motor", rfc);
            telemetry.addData("Lb_Motor", lbc);
            telemetry.addData("Rb_Motor", rbc);
            
            // ****** Arm Motor Control ****** //
            //Get game pad vaules
            //D_pad_up = gamepad2.dpad_up;
            //D_pad_down = gamepad2.dpad_down;
            armStick = gamepad2.left_stick_y;
            reset_X = gamepad2.x;
            reset_A = gamepad2.a;
            tuck_Y = gamepad2.y;
            untuck_B = gamepad2.b;
            
            // Button by power and not button by position
            if (armStick < 0){  // moving arm down
                if (!armInPowerMode) {
                    Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armInPowerMode = true;
                }
                armPlace = Arm_Motor.getCurrentPosition();
                if (armPlace > armPositionToHighPower){
                    Arm_Motor.setPower(armUpPower * armStick);
                }
                else{
                    Arm_Motor.setPower(armDownPower * armStick * -1);
                }
            } else if (armStick > 0){  // moving arm up
                if (!armInPowerMode) {
                    Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armInPowerMode = true;
                }
                Arm_Motor.setPower(armUpPower * armStick);
                armPlace = Arm_Motor.getCurrentPosition();
            } else {
                if(armInPowerMode) {
                    Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armInPowerMode = false;
                }
                Arm_Motor.setPower(armHoldPower);
                Arm_Motor.setTargetPosition(armPlace);
            }
            
            
           // auto arm positioning code  
            if ((gamepad2.dpad_up || gamepad2.dpad_down) && !lastDPadState) {
                armPresetPos = getPositionPreset(armPresetPos);
                switch(armPresetPos){
                    case 1:
                        //lowest position
                        armPlace = 0;
                        futureWristPlace = 0.525;
                        futureWristTime = time + futureSmallWristDelay;
                        futureWristReady = true;
                        break;
                    case 2:
                        //low pole
                        armPlace = 205;
                        futureWristPlace = 0.283;
                        futureWristTime = time + futureBigWristDelay;
                        futureWristReady = true;
                        break;
                    case 3:
                        //mid pole
                        armPlace = 360;
                        futureWristPlace = 0.084;
                        futureWristTime = time + futureWristDelay;
                        futureWristReady = true;
                        break;
                    case 4:
                        //high pole
                        armPlace = 451;
                        futureWristPlace = 0.184;
                        futureWristTime = time + futureWristDelay;
                        futureWristReady = true;
                        break;
                }
                Arm_Motor.setTargetPosition(armPlace);
                
            }
            lastDPadState = (gamepad2.dpad_up || gamepad2.dpad_down);
            
            //arm tucking code
            if (tuck_Y && !lastTuck_Y){  // Tuck time
                armPlace = 0;
                Arm_Motor.setTargetPosition(armPlace);
                wristPlace = 1;
                Wrist_Servo.setPosition(1);
                armPresetPos = 0;
            }
            lastTuck_Y = tuck_Y;
            
            //arm untucking code
            if (untuck_B && !lastUntuck_B){  // Untuck time
                armPlace = 360;
                futureWristPlace = 0.084;
                futureWristTime = time + futureWristDelay;
                futureWristReady = true;
                armPresetPos = 3;
            }
            lastUntuck_B = untuck_B;
            
            
            
            //arm reset code
            if ((armStick < -0.75) && reset_X && reset_A){
                currentArmPosition = Arm_Motor.getCurrentPosition();
                if((lastArmPosition == currentArmPosition) && (currentArmPosition<armResetLimit)){
                    Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armPlace = 0;
                    Arm_Motor.setTargetPosition(armPlace);
                    Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                lastArmPosition = currentArmPosition;
            }
            
            //Arm telemetry
            //telemetry.addData("Arm Speed", armUpPower);
            telemetry.addData("Arm Set Point", armPlace);
            telemetry.addData("Real Arm Position", Arm_Motor.getCurrentPosition());
            telemetry.addData("Arm In Power Mode", armInPowerMode );
            telemetry.addData("ArmStick",armStick );
            telemetry.addData("ArmPresetPos",armPresetPos);
            //telemetry.addData("D_pad_up",D_pad_up );
            //telemetry.addData("D_pad_down", D_pad_down )
            
            // ****** Gripper Control ****** //
            // Get Control Values
            wristStick = -gamepad2.right_stick_y; //Negative so that it goes in the right direction 
            clawButton1 = gamepad1.right_trigger;
            clawButton2 = gamepad2.right_trigger;
            
            //Control wrist
            wristPlace = wristPlace-(wristAdd*wristStick);
            wristPlace = Range.clip(wristPlace,0,1);
            if(futureWristReady && (time >= futureWristTime)){
                wristPlace = futureWristPlace;
                futureWristReady = false;
            }
            Wrist_Servo.setPosition(wristPlace);
            
            //Control Claw
            if(clawButton1>.5 || clawButton2>.5){
                clawPosition=1;
            }
            else{
                clawPosition=-1;
            }
            Lg_Servo.setPosition(clawPosition);
            Rg_Servo.setPosition(-clawPosition);
            
            //Gripper Telemetry
            telemetry.addData("wrist position", wristPlace);
            telemetry.addData("claw position", clawPosition);
            
            telemetry.update();
            
            
        }
    }
    
    private int getPositionPreset(int curr) {
        if (gamepad2.dpad_up) {
            return Range.clip(curr-1, 1, 4);
        } else if (gamepad2.dpad_down) {
            return Range.clip(curr+1, 1, 4);
        }
        return curr;
    }
}
