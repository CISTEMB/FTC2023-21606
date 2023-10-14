/*
Copyright 2023 FIRST Tech Challenge Team 21606

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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class CenterStageTest extends OpMode {
    /* Declare OpMode members. */
    private DcMotor arm_motor;
    private ColorSensor color_Sensor;
    private Blinker control_Hub;
    private Blinker expansion_Hub_1;
    private DcMotor lb_motor;
    private DcMotor lf_motor;
    private Servo lg_Servo;
    private DcMotor rb_motor;
    private DcMotor rf_motor;
    private Servo rg_Servo;
    private Servo wrist_Servo;
    private Servo front_color_wheel;
    private Gyroscope imu;
    private Servo rear_color_wheel;


    @Override
    public void init() {
        arm_motor = hardwareMap.get(DcMotor.class, "Arm-motor");
        color_Sensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        lb_motor = hardwareMap.get(DcMotor.class, "Lb-motor");
        lf_motor = hardwareMap.get(DcMotor.class, "Lf-motor");
        lg_Servo = hardwareMap.get(Servo.class, "Lg_Servo");
        rb_motor = hardwareMap.get(DcMotor.class, "Rb-motor");
        rf_motor = hardwareMap.get(DcMotor.class, "Rf-motor");
        rg_Servo = hardwareMap.get(Servo.class, "Rg_Servo");
        wrist_Servo = hardwareMap.get(Servo.class, "Wrist_Servo");
        front_color_wheel = hardwareMap.get(Servo.class, "front_color_wheel");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        rear_color_wheel = hardwareMap.get(Servo.class, "rear_color_wheel");
        telemetry.addData("Status", "Initialized");
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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
