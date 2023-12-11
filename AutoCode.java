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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/*
 * Add a line that says "@Disabled" line to remove this OpMode from the Driver Station OpMode list
 */

@Autonomous(name="Auto Code", group="Practice", preselectTeleOp="Teleop Code")

public class AutoCode extends OpMode
{
    private RobotHardware robot = new RobotHardware(this);
    
    // Declare general global variables {
    private ElapsedTime runtime = new ElapsedTime();
    private double wristPlace = 0;
    private int elbowHold = 0;
    private double waitTime;
    // }
    
    // Declare Park variables { 
    private boolean parkRight = false;
    private boolean parkLeft = false;
    private boolean parkFrozen = false;
    // }

    // Declare auto state machine enums and variables {
    private enum DriveState {
        INIT,
        CLEAR_TRUSS,
        FIND_LINE,
        CENTER_PUSH_PROP,
        CENTER_MOVE_BACK,
        LEFT_MOVE_BACK,
        LEFT_STRAFE_TO_LINE,
        RIGHT_MOVE_BACK,
        RIGHT_STRAFE_TO_LINE,
        DROP_STEP1,
        DROP_STEP2,
        DROP_STEP3,
        DROP_STEP4,
        DROP_STEP0_PREPICKUP,
        PARK_RIGHT,
        PARK_LEFT,
        END,
        TEST_TURN
    };
    
    private DriveState CurrentDriveState;
    private boolean InitDriveState = false;
    private ElapsedTime DriveStateTime = new ElapsedTime();
    // } 
    
    // Declare distance sensor stuff {
    private double minReadingLeft = 65535;
    private double minReadingRight = 65535;
    private double minReadingCenter = 65535;
    private boolean propLeft = false;
    private boolean propRight = false;
    private boolean propCenter = false;
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
        robot.setWristPosition(wristPlace);
        newDriveState(DriveState.INIT);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        
        // Select Part Location
        if(gamepad1.b && !parkFrozen) {
            parkRight = true;
            parkLeft = false;
        }
        if(gamepad1.x && !parkFrozen) {
            parkRight = false;
            parkLeft = true;
        }
        if(gamepad1.a && !parkFrozen) {
            parkRight = false;
            parkLeft = false;
        }
        if(gamepad1.guide) {
            parkFrozen = !parkFrozen;
        }
        telemetry.addData("parkState","parkRight: " + parkRight + " parkLeft: " + parkLeft + " parkFrozen: " + parkFrozen);
        telemetry.update();
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

        // Main State Machine Loop
        switch (CurrentDriveState){
            case INIT:
                telemetry.addData("Drive state", "Init");
                if (InitDriveState)
                {
                    robot.setDrivePower(0, 0, 0, 0);
                    
                    InitDriveState = false;
                } 
                if (true) { // Because we not want to immedatly start
                    newDriveState(DriveState.TEST_TURN);
                }
                break;
    
            case TEST_TURN:
                telemetry.addData("Drive state", "Test turn");
                if (InitDriveState) {
                    turn(0.5,-0.5);
                    InitDriveState = false;
                }
                if (gotAngle(-45,true)) {
                    robot.setDrivePower(0,0,0,0);
                    newDriveState(DriveState.END);
                }
                break;
            
            case CLEAR_TRUSS:
                telemetry.addData("Drive state", "Clearing Truss");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(.5);
                    InitDriveState = false;
                } 
                if (gotDistance (24, true)) {  // Time to leave
                    newDriveState(DriveState.FIND_LINE);
                } else {  // Stick around
                    
                }
                break;
                
            case FIND_LINE:
                telemetry.addData("Drive state","Find Line");
                if (InitDriveState)  // Start Starting
                {
                    speedDrive(.075); 
                    InitDriveState = false;
                } else if (gotLine()) {  // Time to leave
                    if (propCenter) {
                        newDriveState(DriveState.CENTER_PUSH_PROP);
                    } else if (propLeft) {
                        newDriveState(DriveState.LEFT_MOVE_BACK);
                    } else if (propRight) {
                        newDriveState(DriveState.RIGHT_MOVE_BACK);
                    }
                    else {
                        newDriveState(DriveState.END);
                    }
                } else if (gotDistance(55, true)){
                    newDriveState(DriveState.END);
                }
                else {        // Stick around
                    checkLowestDSensor();
                }
                break;
                
            case CENTER_PUSH_PROP:
                telemetry.addData("Drive state", "Center Push Prop");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(.1);
                    InitDriveState = false;
                } 
                if (gotDistance (5, true)) {  // Time to leave
                    newDriveState(DriveState.CENTER_MOVE_BACK);
                } else {  // Stick around
                    
                }
                break;    
                
            case CENTER_MOVE_BACK:
                telemetry.addData("Drive state", "Center move back");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(-.1);
                    InitDriveState = false;
                } 
                if (gotDistance (-9, false)) {  // Time to leave
                    speedDrive(0);
                    newDriveState(DriveState.DROP_STEP0_PREPICKUP);
                } else {  // Stick around
                    
                }
                break;    
                
            case LEFT_MOVE_BACK:
                telemetry.addData("Drive state", "Left move back");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(-.1);
                    InitDriveState = false;
                } 
                if (gotDistance (-9, false)) {  // Time to leave
                    speedDrive(0);
                    newDriveState(DriveState.LEFT_STRAFE_TO_LINE);
                } else {  // Stick around
                    
                }
                break;
                
            case LEFT_STRAFE_TO_LINE:
                telemetry.addData("Drive state", "Left Strafe to Line");
                if (InitDriveState)  // Start Starting
                {
                    robot.resetDriveEncoders();
                    speedStrafe(-.05); 
                    InitDriveState = false;
                } else if (gotLine()) {  // Time to leave
                        stopDrive();
                        newDriveState(DriveState.DROP_STEP0_PREPICKUP);
                    
                } else if (gotStrafeDistance(-10, false)){ // todo make 40
                    newDriveState(DriveState.END);
                }
                else {        // Stick around
                }
                break;
                
            case RIGHT_MOVE_BACK:
                telemetry.addData("Drive state", "Right move back");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(-.1);
                    InitDriveState = false;
                } 
                if (gotDistance (-9, false)) {  // Time to leave
                    speedDrive(0);
                    newDriveState(DriveState.RIGHT_STRAFE_TO_LINE);
                } else {  // Stick around
                    
                }
                break;    
                
            case RIGHT_STRAFE_TO_LINE:
                telemetry.addData("Drive state", "Right Strafe to Line");
                if (InitDriveState)  // Start Starting
                {   
                    robot.resetDriveEncoders();
                    speedStrafe(.05); 
                    InitDriveState = false;
                } else if (gotLine()) {  // Time to leave
                        stopDrive();
                        newDriveState(DriveState.DROP_STEP0_PREPICKUP);
                    
                } else if (gotStrafeDistance(10, true)){ 
                    newDriveState(DriveState.END);
                }
                else {        // Stick around
                }
                break;
        
            case DROP_STEP1://{
                telemetry.addData("Arm state", "Drop step 1");
                if (InitDriveState) {
                     waitTime = DriveStateTime.milliseconds()+ 1000;
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitDriveState = false;
                }
                if (robot.elbowWithinRange(robot.ELBOW_PICKUP) &&
                    DriveStateTime.milliseconds() > waitTime) {
                    newDriveState(DriveState.DROP_STEP2);
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_PICKUP, robot.WRIST_PICKUP, .02);
                }
                break;//}
                
            case DROP_STEP0_PREPICKUP:
                telemetry.addData("Arm state", "Drop Step 0: PrePickup");
                if (InitDriveState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitDriveState = false;
                }
                if (robot.elbowWithinRange(robot.ELBOW_PREPICKUP)){
                    newDriveState(DriveState.DROP_STEP1);
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_PREPICKUP, robot.WRIST_PICKUP, 0.04);
                    //gripperControl(grip_wide_btn, grip_mid_btn);
                }
                break;//}
        
            case DROP_STEP2://{
                telemetry.addData("Arm state", "Drop step 2");
                if (InitDriveState) {
                    waitTime = DriveStateTime.milliseconds()+ 1000;
                    robot.setGripperPosition(robot.LEFT_GRIP_OPEN, robot.RIGHT_GRIP_OPEN);
                    InitDriveState = false;
                }
                if (DriveStateTime.milliseconds() > waitTime) {
                    newDriveState(DriveState.DROP_STEP3);
                } else {
                    
                }
                break;//}
                
            case DROP_STEP3://{
                telemetry.addData("Arm state", "Drop step 3");
                if (InitDriveState) {
                    waitTime = DriveStateTime.milliseconds()+ 1000;
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitDriveState = false;
                }
                if (robot.elbowWithinRange(robot.ELBOW_PRETUCK) &&
                    DriveStateTime.milliseconds() > waitTime) {
                    newDriveState(DriveState.DROP_STEP4);
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_PRETUCK, wristPlace, .02);
                }
                break;//}
                
            case DROP_STEP4://{
                telemetry.addData("Arm state", "Drop step 4");
                if (InitDriveState) {
                    waitTime = DriveStateTime.milliseconds()+ 1000;
                    robot.setGripperPosition(robot.LEFT_GRIP_CLOSED, robot.RIGHT_GRIP_CLOSED);
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitDriveState = false;
                }
                if (robot.elbowWithinRange(robot.ELBOW_TUCK) &&
                    DriveStateTime.milliseconds() > waitTime) {
                    if(parkRight){
                        newDriveState(DriveState.PARK_RIGHT);
                    } else if(parkLeft){
                        newDriveState(DriveState.PARK_LEFT);
                    } else{
                        newDriveState(DriveState.END);  
                    }
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_TUCK, robot.WRIST_TUCK, .02);
                }
                break;//}
            
            case PARK_RIGHT:
                telemetry.addData("Drive state", "Park Right");
                if (InitDriveState)  // Start Starting
                {   
                    robot.resetDriveEncoders();
                    speedStrafe(.1); 
                    InitDriveState = false;
                } else if (gotStrafeDistance(50, true) || robot.rightd_sensor.getDistance(DistanceUnit.INCH)<=3){ 
                    newDriveState(DriveState.END);
                }
                else {        // Stick around
                }
                break;
                
            case PARK_LEFT:
                telemetry.addData("Drive state", "Park Left");
                if (InitDriveState)  // Start Starting
                {   
                    robot.resetDriveEncoders();
                    speedStrafe(-.1); 
                    InitDriveState = false;
                } else if (gotStrafeDistance(-50, false) || robot.leftd_sensor.getDistance(DistanceUnit.INCH)<=3){ 
                    newDriveState(DriveState.END);
                }
                else {        // Stick around
                }
                break;
                
            case END:
                telemetry.addData("Drive state","END");
                if (true)
                {
                    stopDrive();
                }
                break;
        }
        
        // Telemetry Update.
        telemetry.addData("Angle", robot.getAngle());
        telemetry.addData("Distances", "left: (%.2f); center: (%.2f); right: (%.2f);", minReadingLeft, minReadingCenter, minReadingRight);
        telemetry.addData("Prop Position", " left: " + propLeft + " center: " + propCenter + " right: " + propRight );
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

    private void speedDrive (double speed){
        robot.lf_motor.setPower(speed);
        robot.rf_motor.setPower(speed);
        robot.lb_motor.setPower(speed);
        robot.rb_motor.setPower(speed);
        robot.lf_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void speedStrafe (double speed){
        robot.lf_motor.setPower(speed);
        robot.rf_motor.setPower(-speed);
        robot.lb_motor.setPower(-speed);
        robot.rb_motor.setPower(speed);
        robot.lf_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void stopDrive (){
        robot.lf_motor.setPower(0);
        robot.rf_motor.setPower(0);
        robot.lb_motor.setPower(0);
        robot.rb_motor.setPower(0);
        robot.lf_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private boolean gotDistance (double inches, boolean dirForward){
        int counts=(int)(inches*38);
        if (dirForward){
            if ((robot.lf_motor.getCurrentPosition() +
                 robot.rf_motor.getCurrentPosition() +
                 robot.lb_motor.getCurrentPosition() +
                 robot.rb_motor.getCurrentPosition())>= counts*4){
                    return true;
                }else{
                    return false;
                }
        } else {
            if ((robot.lf_motor.getCurrentPosition() +
                 robot.rf_motor.getCurrentPosition() +
                 robot.lb_motor.getCurrentPosition() +
                 robot.rb_motor.getCurrentPosition())<= counts*4){
                    return true;
                }else{
                    return false;
                }
        }
    }

    private boolean gotStrafeDistance (double inches, boolean dirRight){
        int counts=(int)(inches*67);
        if (dirRight){
            if ((robot.lf_motor.getCurrentPosition() +
                 -robot.rf_motor.getCurrentPosition() +
                 -robot.lb_motor.getCurrentPosition() +
                 robot.rb_motor.getCurrentPosition())>= counts*4){
                    return true;
                }else{
                    return false;
                }
        } else {
            if ((robot.lf_motor.getCurrentPosition() +
                 -robot.rf_motor.getCurrentPosition() +
                 -robot.lb_motor.getCurrentPosition() +
                 robot.rb_motor.getCurrentPosition())<= counts*4){
                    return true;
                }else{
                    return false;
                }
        }
    }
    
    private boolean gotAngle(double degrees, boolean clockwise) {
        double heading = robot.getAngle();
        if (clockwise) {
           return (heading <= degrees);
        } else {
            return (heading >= degrees);
        }
    }

    private void turn(double leftSpeed, double rightSpeed) {
        robot.setDrivePower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }
    
    private boolean gotLine(){
        int blue = robot.clrl_sensor.blue();
        int red = robot.clrl_sensor.red();
        telemetry.addData("color", "red:"+red+" blue:"+blue);
        if (blue > robot.BLUE_LIMIT){
            return true;
        } else if (red > robot.RED_LIMIT){
            return true;
        } else {
            return false;
        }
    }
    
    private void checkLowestDSensor() {
        double tempL = robot.leftd_sensor.getDistance(DistanceUnit.INCH);
        double tempR = robot.rightd_sensor.getDistance(DistanceUnit.INCH);
        if (tempL < minReadingLeft) {
            minReadingLeft = tempL;
        }
        if (tempR < minReadingRight) {
            minReadingRight = tempR;
        }
        if (minReadingLeft < minReadingRight && minReadingLeft < 8) {
            propLeft = true;
            propRight = false;
            propCenter = false;
        } else if (minReadingRight < minReadingLeft && minReadingRight < 8) {
            propRight = true;
            propLeft = false;
            propCenter = false;
        } else {
            propCenter = true;
            propRight = false;
            propLeft = false;
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