/*
Copyright 2022 FIRST Tech Challenge Team 21616

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
@TeleOp(name="Test:Motor", group="Test")

public class TestMotor extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_1;
    private DcMotor lb_motor;
    private DcMotor lf_motor;
    private DcMotor rb_motor;
    private DcMotor rf_motor;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        lb_motor = hardwareMap.get(DcMotor.class, "Lb-motor");
        lf_motor = hardwareMap.get(DcMotor.class, "Lf-motor");
        rb_motor = hardwareMap.get(DcMotor.class, "Rb-motor");
        rf_motor = hardwareMap.get(DcMotor.class, "Rf-motor");
        
        lb_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rb_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lf_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        rf_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        

        telemetry.addData("Status", "Initialized");
        telemetry.addData("MotorRf","Press Y");
        telemetry.addData("MotorLf","Press X");
        telemetry.addData("MotorRb","Press A");
        telemetry.addData("MotorLb","Press B");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // testing motor rf
            if(gamepad1.y){
                rf_motor.setPower(0.25);
                telemetry.addData("MotorRf","Running");
                
            } else {
                rf_motor.setPower(0);
                telemetry.addData("MotorRf","Press Y");
                
            }

        // testing motor lf
            if (gamepad1.x){
                lf_motor.setPower(0.25);
                telemetry.addData("MotorLf","Running" );
            } else {
                lf_motor.setPower(0);
                telemetry.addData("MotorLf","Press X" );
            }
            
            // testing motor rb
            if (gamepad1.a){
                rb_motor.setPower(0.25);
                telemetry.addData("MotorRb","Running" );
            } else {
                rb_motor.setPower(0);
                telemetry.addData("MotorRb","Press A" );
            }
            
            // testing motor lb
            if (gamepad1.b){
                lb_motor.setPower(0.25);
                telemetry.addData("MotorLb","Running" );
            } else {
                lb_motor.setPower(0);
                telemetry.addData("MotorLb","Press B" );
            }
                

            telemetry.update();
        }
    }
}

