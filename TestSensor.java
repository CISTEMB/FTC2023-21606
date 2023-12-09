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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Add a line that says "@Disabled" line to remove this OpMode from the Driver Station OpMode list
 */

@TeleOp(name="Test Sensor", group="Test")

public class TestSensor extends OpMode
{
    private RobotHardware robot = new RobotHardware(this);
    
    // Declare controller related variables(prev button presses, etc.) {
    private boolean reset_btn = false;
    
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
        // runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        reset_btn = gamepad1.a;
        if (reset_btn) {
            robot.elbow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lf_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lb_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rf_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rb_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        
        // all of le telemetry
        telemetry.addData("Reset encoders button", "A");
        telemetry.addData("Elbow Location", robot.elbow_motor.getCurrentPosition());
        telemetry.addData("Wheels", "lf_motor: (%.2f); rf_motor: (%.2f); lb_motor: (%.2f); rb_motor: (%.2f)", robot.lf_motor.getCurrentPosition(), robot.rf_motor.getCurrentPosition(), robot.lb_motor.getCurrentPosition(), robot.rb_motor.getCurrentPosition());
        telemetry.addData("Distances", "left: (%.2f); right: (%.2f);", robot.leftd_sensor.getDistance(DistanceUnit.INCH), robot.rightd_sensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Colors", "red: (%d); green: (%d); blue: (%d)", robot.clrl_sensor.red(), robot.clrl_sensor.green(), robot.clrl_sensor.blue());
        
        
        // WIll this work
        /* telemetry.addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
                
        telemetry.addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                });
        telemetery.addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                }); */
       
        
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
