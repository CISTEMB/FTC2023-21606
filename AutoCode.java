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
    
    private ElapsedTime runtime = new ElapsedTime();
    private double wristPlace = 0;
    private int elbowHold = 0;
    private double waitTime;
    
    private boolean side_btn;                // current btn state
    private boolean side_btn_old = false;    // prev btn state
    private boolean side_btn_press = false;  // when btn changes from false to true
    private boolean color_btn;
    private boolean color_btn_old = false;
    private boolean color_btn_press = false;
    private boolean back_pixel_btn = false;
    private boolean back_pixel_btn_old= false;
    private boolean back_pixel_btn_press = false;
    private boolean park_btn = false;
    private boolean park_btn_old = false;
    private boolean park_btn_press = false;
    
    private boolean backPixel = false;
    private boolean park = false; 

    private enum Color {
        RED,
        BLUE
    }
    private Color colorSetting = Color.RED;
    
    private enum Side {
        BACKDROP,
        AIRSTRIP
    }
    private Side sideSetting = Side.BACKDROP;
    
    private enum Prop {
        LEFT,
        RIGHT,
        CENTER,
        UNKNOWN
    }
    private Prop propLocation = Prop.UNKNOWN;
    
    // Declare drive state machine enums and variables {
    private enum DriveState {
        INIT,
        WRIST_DOWN_5,
        CLEAR_TRUSS_10,
        FIND_LINE_20,
        CENTER_PUSH_PROP, //nn
        CENTER_MOVE_BACK, //30
        LEFT_MOVE_BACK, //40
        LEFT_STRAFE_TO_LINE, //50
        LEFT_MOVE_BACK_AGAIN, //nn
        RIGHT_MOVE_BACK,  //common
        RIGHT_STRAFE_TO_LINE, //common
        RIGHT_MOVE_BACK_AGAIN, //nn
        POSITION_PIXEL_60,
        SLIGHT_DROP_70,
        SLIGHT_RAISE_80,
        DROP_STEP0_PREPICKUP,
        DROP_STEP1,
        DROP_STEP2,
        DROP_STEP3,
        DROP_STEP4,
        TURN_RIGHT,
        DRIVE_TO_BACKBOARD1,
        DRIVE_TO_BACKBOARD2,
        BACKBOARD_MOVE_CENTER,
        BACKBOARD_MOVE_RIGHT,
        BACKBOARD_MOVE_LEFT,
        DROP_BACKBOARD_0_PREPICKUP,
        PARK_RIGHT,
        PARK_LEFT,
        END,
        // TEST_TURN,
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
    private double backboardCenterMin = 0;
    private double backboardCenterMax = 318727981;
    private double backboardLeftMin = 0;
    private double backboardLeftMax = 318727981;
    private double backboardRightMin = 0;
    private double backboardRightMax = 318727981;
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
        // Initialize states for state machines {
        newDriveState(DriveState.INIT);
        //newArmState(ArmState.ARM_STATE_INIT);
        // }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        back_pixel_btn = gamepad2.a;
        if (!back_pixel_btn_old && back_pixel_btn) {
            back_pixel_btn_press = true;
        } else {
            back_pixel_btn_press = false;
        }
        back_pixel_btn_old = back_pixel_btn;
        
        color_btn = gamepad2.y;
        if (!color_btn_old && color_btn) {
            color_btn_press = true;
        } else {
            color_btn_press = false;
        }
        color_btn_old = color_btn;
        
        park_btn = gamepad2.x;
        if (!park_btn_old && park_btn) {
            park_btn_press = true;
        } else {
            park_btn_press = false;
        }
        park_btn_old = park_btn;
        
        side_btn = gamepad2.b;
        if (!side_btn_old && side_btn) {
            side_btn_press = true;
        } else {
            side_btn_press = false;
        }
        side_btn_old = side_btn;
        
        if(back_pixel_btn_press) {
            backPixel = !backPixel;
        }
        
        if(park_btn_press) {
            park = !park;
        }
        
        if(color_btn_press) {
            switch(colorSetting) {
                case RED:
                    colorSetting = Color.BLUE;
                    break;            
                case BLUE:
                    colorSetting = Color.RED;
                    break;
            }
        }
        
        if(side_btn_press) {
            switch(sideSetting) {
                case BACKDROP:
                    sideSetting = Side.AIRSTRIP;
                    break;            
                case AIRSTRIP:
                    sideSetting = Side.BACKDROP;
                    break;
            }
        }
        
        
        telemetryConfig();
        telemetry.addData("----Status----","");
        telemetry.addData("Gyro",robot.getAngle());
        telemetry.update();
    }
    
    public void telemetryConfig() {
        telemetry.addData("----Configuration----","");
        telemetry.addData("(Y) Color", colorSetting==Color.RED ? "Red" : "Blue");
        telemetry.addData("(B) Side", sideSetting==Side.BACKDROP ? "Backdrop" : "Airstrip");
        telemetry.addData("(A) Back pixel", backPixel ? "Yes" : "No");
        telemetry.addData("(X) Parking", park ? "Yes" : "No");
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
        
       switch (CurrentDriveState){
            case INIT:
                telemetry.addData("Drive state", "Init");
                if (InitDriveState)
                {
                    robot.setDrivePower(0, 0, 0, 0);
                    
                    InitDriveState = false;
                } 
                if (true) { // Because we not want to immedatly start
                    // newDriveState(DriveState.TEST_TURN);
                    newDriveState(DriveState.WRIST_DOWN_5);
                }
                break;
            
            case WRIST_DOWN_5://{
                telemetry.addData("Drive state", "Wrist Down_5");
                if (InitDriveState) {
                    waitTime = DriveStateTime.milliseconds() + 2000;
                    robot.wrist_servo.setPosition(robot.WRIST_PICKUP);
                    InitDriveState = false;
                }
                if (DriveStateTime.milliseconds() > waitTime) {
                    newDriveState(DriveState.CLEAR_TRUSS_10);
                } else {
                    
                }
                break;//}
            
            case CLEAR_TRUSS_10:
                telemetry.addData("Drive state", "Clearing Truss_10");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(.5);
                    InitDriveState = false;
                } 
                if (gotDistance (28, true)) {  // Time to leave
                    newDriveState(DriveState.FIND_LINE_20);
                } else {  // Stick around
                    
                }
                break;
                
            case FIND_LINE_20:
                telemetry.addData("Drive state","Find Line");
                if (InitDriveState)  // Start Starting
                {
                    speedDrive(.075); 
                    InitDriveState = false;
                } else if (gotLine()) {  // Time to leave
                    switch (propLocation) {
                        case CENTER:
                            newDriveState(DriveState.CENTER_PUSH_PROP);
                            break;
                        case LEFT:
                            newDriveState(DriveState.LEFT_MOVE_BACK);
                            break;
                        case RIGHT:
                            newDriveState(DriveState.RIGHT_MOVE_BACK);
                            break;
                        default:
                            newDriveState(DriveState.END);
                    }
                    
                    /* if (propCenter) {
                        newDriveState(DriveState.CENTER_PUSH_PROP);
                    } else if (propLeft) {
                        newDriveState(DriveState.LEFT_MOVE_BACK);
                    } else if (propRight) {
                        newDriveState(DriveState.RIGHT_MOVE_BACK);
                    }
                    else {
                        newDriveState(DriveState.END);
                    } */
                } else if (gotDistance(45, true)){
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
                
            case  LEFT_MOVE_BACK:
                telemetry.addData("Drive state", "Left move back");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(-.1);
                    InitDriveState = false;
                } 
                if (gotDistance (-7, false)) {  // Time to leave
                    speedDrive(0);
                    newDriveState(DriveState.LEFT_STRAFE_TO_LINE);
                } else {  // Stick around
                    
                }
                break;    
            case LEFT_STRAFE_TO_LINE:
                telemetry.addData("Drive state","Left Strafe to Line");
                if (InitDriveState)  // Start Starting
                {
                    robot.resetDriveEncoders();
                    speedStrafe(-.05); 
                    InitDriveState = false;
                } else if (gotLine()) {  // Time to leave
                        stopDrive();
                        newDriveState(DriveState.LEFT_MOVE_BACK_AGAIN);
                    
                } else if (gotStrafeDistance(-10, false)){ // todo make 40
                    newDriveState(DriveState.END);
                }
                else {        // Stick around
                }
                break;
                
        case  LEFT_MOVE_BACK_AGAIN:
                telemetry.addData("Drive state", "Left move back again");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(-.1);
                    InitDriveState = false;
                } 
                if (gotDistance (-2, false)) {  // Time to leave
                    speedDrive(0);
                    newDriveState(DriveState.DROP_STEP0_PREPICKUP);
                } else {  // Stick around
                    
                }
                break;
                
            case  RIGHT_MOVE_BACK:
                telemetry.addData("Drive state", "Right move back");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(-.1);
                    InitDriveState = false;
                } 
                if (gotDistance (-7, false)) {  // Time to leave
                    speedDrive(0);
                    newDriveState(DriveState.RIGHT_STRAFE_TO_LINE);
                } else {  // Stick around
                    
                }
                break;    
                
            case RIGHT_STRAFE_TO_LINE:
                telemetry.addData("Drive state","Right Strafe to Line");
                if (InitDriveState)  // Start Starting
                {   
                    robot.resetDriveEncoders();
                    speedStrafe(.20); 
                    InitDriveState = false;
                } else if (gotLine()) {  // Time to leave
                        stopDrive();
                        newDriveState(DriveState.POSITION_PIXEL_60);
                    
                } else if (gotStrafeDistance(10, true)){ 
                    newDriveState(DriveState.END);
                }
                else {        // Stick around
                }
                break;
                
        case  RIGHT_MOVE_BACK_AGAIN:
                telemetry.addData("Drive state", "right move back again");
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    speedDrive(-.1);
                    InitDriveState = false;
                } 
                if (gotDistance (-2, false)) {  // Time to leave
                    speedDrive(0);
                    newDriveState(DriveState.END);
                    //newDriveState(DriveState.DROP_STEP0_PREPICKUP);
                } else {  // Stick around
                    
                }
                break;
        case POSITION_PIXEL_60:
                telemetry.addData("Drive state","Position Pixel_60");
                if (InitDriveState)  // Start Starting
                {
                    robot.resetDriveEncoders();
                    if(propLocation == Prop.RIGHT) {
                        speedStrafe(.20);
                    } else {
                        speedStrafe(-.20);
                    }
                    InitDriveState = false;
                } else if (((propLocation == Prop.RIGHT) && (gotStrafeDistance(1.5,true))) ||
                           ((propLocation == Prop.LEFT) && (gotStrafeDistance(-1.5,false)))) {
                    speedDrive(0);           
                    newDriveState(DriveState.SLIGHT_DROP_70);
                }
                else {        // Stick around
                }
                break;
                
            case SLIGHT_DROP_70://{
                telemetry.addData("Drive state", "Slight Drop_70");
                if (InitDriveState) {
                    waitTime = DriveStateTime.milliseconds() + 1000;
                    robot.lg_servo.setPosition(robot.LEFT_GRIP_SLIGHT_OPEN);
                    InitDriveState = false;
                }
                if (DriveStateTime.milliseconds() > waitTime) {
                    newDriveState(DriveState.SLIGHT_RAISE_80);
                } else {
                    
                }
                break;//}    
            
            case SLIGHT_RAISE_80:
                telemetry.addData("Drive state", "Slight Raise_80");
                if (InitDriveState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitDriveState = false;
                }
                if (robot.elbowWithinRange(robot.ELBOW_SLIGHT_RAISE)){
                    newDriveState(DriveState.END);
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_SLIGHT_RAISE, robot.WRIST_PICKUP, 0.04);
                    //gripperControl(grip_wide_btn, grip_mid_btn);
                }
                break;//}
            
                
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
                telemetry.addData("Arm state", "Drop Step 0; Prepickup");
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
                    // TOFIX robot.setGripperPosition(robot.LEFT_GRIP_OPEN, robot.RIGHT_GRIP_OPEN);
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
                    // TOFIX robot.setGripperPosition(robot.LEFT_GRIP_CLOSED, robot.RIGHT_GRIP_CLOSED);
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitDriveState = false;
                }
                if (robot.elbowWithinRange(robot.ELBOW_TUCK) &&
                    DriveStateTime.milliseconds() > waitTime) {
                    if(0==1){
                        newDriveState(DriveState.PARK_RIGHT);
                    } else if(0==1){
                        newDriveState(DriveState.PARK_LEFT);
                    } else{
                        newDriveState(DriveState.END);  
                    }
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_TUCK, robot.WRIST_TUCK, .02);
                }
                break;//}
            case TURN_RIGHT:
                telemetry.addData("Drive state", "Turn Right");
                if (InitDriveState) {
                    turn(0.5, -0.5);
                    InitDriveState = false;
                }
                if (true) {
                    turn(0.5, -0.5);
                    if (gotAngle(90,true)) {
                        robot.setDrivePower(0, 0, 0, 0);
                        newDriveState(DriveState.DRIVE_TO_BACKBOARD1);
                    } else {
                        
                    }
                }
            case DRIVE_TO_BACKBOARD1:
                telemetry.addData("Drive state", "Drive to Backboard 1");
                if (InitDriveState) {
                    robot.gyroDrive(90, 0.5);
                    InitDriveState = false;
                }
                if (true) {
                    robot.gyroDrive(90, 0.5);
                    int blue = robot.clrl_sensor.blue();
                    int red = robot.clrl_sensor.red();
                    telemetry.addData("color", "red: " + red + " blue: " + blue);
                    if (blue > robot.BLUE_LIMIT || red > robot.RED_LIMIT) {
                        newDriveState(DriveState.DRIVE_TO_BACKBOARD2);
                    } else {
                        
                    }
                }
                break;
            case DRIVE_TO_BACKBOARD2:
                telemetry.addData("Drive state", "Drive to Backboard 2");
                if (InitDriveState) {
                    robot.gyroDrive(-90, 0.5);
                    InitDriveState = false;
                }
                if (gotDistance(5, true)) {
                    robot.setDrivePower(0, 0, 0, 0);
                    if (propCenter) {
                        newDriveState(DriveState.BACKBOARD_MOVE_CENTER);
                    } else if (propLeft) {
                        newDriveState(DriveState.BACKBOARD_MOVE_LEFT);
                    } else if (propRight) {
                        newDriveState(DriveState.BACKBOARD_MOVE_RIGHT);
                    }
                } else {

                }
            case BACKBOARD_MOVE_CENTER:
                telemetry.addData("Drive state", "Backboard move center");
                if (InitDriveState) {
                    InitDriveState = false;
                }
                if (true) {
                    newDriveState(DriveState.DROP_BACKBOARD_0_PREPICKUP);
                }
            case BACKBOARD_MOVE_LEFT:
                telemetry.addData("Drive state", "Backboard move left");
                if (InitDriveState) {
                    InitDriveState = false;
                }
                if (true) {
                    newDriveState(DriveState.DROP_BACKBOARD_0_PREPICKUP);
                }
            case BACKBOARD_MOVE_RIGHT:
                telemetry.addData("Drive state", "Backboard move right");
                if (InitDriveState) {
                    InitDriveState = false;
                }
                if (true) {
                    newDriveState(DriveState.DROP_BACKBOARD_0_PREPICKUP);
                }
            case DROP_BACKBOARD_0_PREPICKUP:
                telemetry.addData("Arm state", "Backboard Drop 0; Prepickup");
                if (InitDriveState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitDriveState = false;
                }
                if (robot.elbowWithinRange(robot.ELBOW_PREPICKUP)){
                    newDriveState(DriveState.DROP_STEP1);
                } else {
                    armControl(robot.ELBOW_MAX_SPEED, robot.ELBOW_PREPICKUP, robot.WRIST_PICKUP, 0.04);
                }
                break;
            case PARK_RIGHT:
                telemetry.addData("Drive state","Park Right");
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
                telemetry.addData("Drive state","Park Left");
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
        

        // Show the elapsed game time and wheel power.
        robot.updatePersistentTelemetry();
        telemetryConfig();
        
        telemetry.addData("Distances", "left: (%.2f); center: (%.2f); right: (%.2f);", minReadingLeft, minReadingCenter, minReadingRight);
        telemetry.addData("Prop Position", propLocation.name());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("the robot", "i wont hold back"); // YOUR NEXT
        telemetry.update();
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
            propLocation = Prop.LEFT;
        } else if (minReadingRight < minReadingLeft && minReadingRight < 8) {
            propLocation = Prop.RIGHT;
        } else {
            propLocation = Prop.CENTER;
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