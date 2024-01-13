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

//{
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
//}

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
    private boolean park = true; 

    private double targetDistanceFromWall;
    private double distanceFromWall;
    private boolean moveRight = true;
                


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
        WRIST_DOWN_10,
        CLEAR_TRUSS_20,
        FIND_LINE_30,
        MOVE_BACK_40,
        STRAFE_TO_LINE_50,
        POSITION_PIXEL_60,
        SLIGHT_DROP_70,
        SLIGHT_RAISE_80,
        BACKUP_TO_CLEAR_PIXEL_90,
        STRAFE_OUT_FROM_UNDER_TRUSS_RIGHT_100,
        STRAFE_OUT_FROM_UNDER_TRUSS_LEFT_105,
        TUCK_ARM_110,
        TURN_TO_FACE_BACKDROP_120,
        DRIVE_TO_BACKBOARD_FAST_125,
        DRIVE_TO_BACKBOARD_LINE_130,
        POSITION_ARM_FOR_BACKBOARD_140,
        STRAFE_TO_LINE_UP_PIXEL_150,
        FINAL_APPROACH_BACKBOARD_160,
        DROP_PIXEL_170,
        BACK_AWAY_FROM_BACKBOARD_180,
        TUCK_ARM_190,
        STRAFE_TO_SIDE_OF_FIELD_200,
        DRIVE_INTO_CORNER_210,
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
    
    private double backboardBlueLeft = 19;
    private double backboardBlueCenter = 25;
    private double backboardBlueRight = 29;
    
    private double backboardRedLeft = 33;
    private double backboardRedCenter = 28;
    private double backboardRedRight = 23;
    
    private double distFromWall = 123456789;
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
        
        // Get buttons for UI {
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
        } //}

        // Standard pre-init telemetry {
        telemetryConfig();
        telemetry.addData("----Status----","");
        telemetry.addData("Gyro",robot.getAngle());
        telemetry.update(); //}
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
        
        // Drive state state machine
        switch (CurrentDriveState){
            
            case INIT: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState)
                {
                    robot.setDrivePower(0, 0, 0, 0);
                    
                    InitDriveState = false;
                } 
                if (true) { // Because we not want to immedatly start
                    // newDriveState(DriveState.TEST_GYRO_DRIVE);
                    newDriveState(DriveState.WRIST_DOWN_10);
                }
                break; //}            
            
            case WRIST_DOWN_10: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState) {
                    waitTime = DriveStateTime.milliseconds() + 100;
                    robot.wrist_servo.setPosition(robot.WRIST_PICKUP);
                    InitDriveState = false;
                }
                if (DriveStateTime.milliseconds() > waitTime) {
                    newDriveState(DriveState.CLEAR_TRUSS_20);
                } else {
                    
                }
                break; //}
            
            case CLEAR_TRUSS_20: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    robot.gyroDrive(.5, 0, 0.03);
                    InitDriveState = false;
                } 
                if (gotDistance (28, true)) {  // Time to leave
                    newDriveState(DriveState.FIND_LINE_30);
                } else {  // Stick around
                    robot.gyroDrive(.5, 0, 0.03);
                }
                break; //}
                
            case FIND_LINE_30: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState)  // Start Starting
                {
                    robot.gyroDrive(.1, 0, 0.03); 
                    InitDriveState = false;
                } else if (gotLine()) {  // Time to leave
                    newDriveState(DriveState.MOVE_BACK_40);
                } else if (gotDistance(45, true)){
                    newDriveState(DriveState.END);
                }
                else {        // Stick around
                    robot.gyroDrive(.1, 0, 0.03); 
                    checkLowestDSensor();
                }
                break; //}
                          
            case MOVE_BACK_40: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState) // Start starting
                {
                    robot.resetDriveEncoders();
                    robot.gyroDrive(-.2, 0.0, 0.03);
                    InitDriveState = false;
                }
                if (((propLocation == Prop.LEFT) && (gotDistance(-7, false))) ||
                    ((propLocation == Prop.RIGHT) && (gotDistance(-7, false)))) { // Time to leave
                    stopDrive();
                    newDriveState(DriveState.STRAFE_TO_LINE_50);
                } else if ((propLocation == Prop.CENTER) && (gotDistance(-3, false))) { // Time to leave
                    stopDrive();
                    newDriveState(DriveState.SLIGHT_DROP_70);
                } else { // Stick around
                    robot.gyroDrive(-.2, 0.0, 0.03);
                }
                break; //}
            
            case STRAFE_TO_LINE_50: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState) // Start starting
                {
                    robot.resetDriveEncoders();
                    if (propLocation == Prop.LEFT) {
                        robot.gyroStrafe(-.2, 0.0, 0.03);
                    } else {
                        robot.gyroStrafe(.2, 0.0, 0.03);
                    }
                    InitDriveState = false;
                }
                if (gotLine()) {  // Time to leave
                    stopDrive();
                    newDriveState(DriveState.POSITION_PIXEL_60);
                } else if (gotStrafeDistance(15, true)){ 
                    newDriveState(DriveState.END);
                }
                else {        // Stick around
                    if (propLocation == Prop.LEFT) {
                        robot.gyroStrafe(-.2, 0.0, 0.03);
                    } else {
                        robot.gyroStrafe(.2, 0.0, 0.03);
                    }
                }
                break; //}
                
            case POSITION_PIXEL_60: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState)  // Start Starting
                {
                    robot.resetDriveEncoders();
                    robot.gyroStrafe(.2, 0.0, 0.03);
                    InitDriveState = false;
                } else if (gotStrafeDistance(1.5,true)) {
                    stopDrive();           
                    newDriveState(DriveState.SLIGHT_DROP_70);
                }
                else { // Stick around
                    robot.gyroStrafe(.2, 0.0, 0.03);
                }
                break; //}
                
            case SLIGHT_DROP_70: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState) {
                    waitTime = DriveStateTime.milliseconds() + 250;
                    robot.lg_servo.setPosition(robot.LEFT_GRIP_SLIGHT_OPEN);
                    InitDriveState = false;
                }
                if (DriveStateTime.milliseconds() > waitTime) {
                    newDriveState(DriveState.SLIGHT_RAISE_80);
                } else {
                    
                }
                break; //}    
            
            case SLIGHT_RAISE_80: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState) {
                    robot.elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    InitDriveState = false;
                }
                if (robot.elbowWithinRange(robot.ELBOW_SLIGHT_RAISE)){
                    newDriveState(DriveState.BACKUP_TO_CLEAR_PIXEL_90);
                } else {
                    armControl(1, robot.ELBOW_SLIGHT_RAISE, robot.WRIST_PICKUP, 0.04);
                }
                break; //}
                
            case BACKUP_TO_CLEAR_PIXEL_90: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState) { // Start starting
                    robot.resetDriveEncoders();
                    robot.gyroDrive(-.2, 0.0, 0.03);
                    InitDriveState = false;
                }
                if (gotDistance(-5, false)) { // Time to leave
                    stopDrive();
                    if ((colorSetting == Color.RED && sideSetting == Side.BACKDROP && propLocation == Prop.LEFT) ||
                        (colorSetting == Color.BLUE && sideSetting == Side.AIRSTRIP && propLocation == Prop.LEFT)) {
                        newDriveState(DriveState.STRAFE_OUT_FROM_UNDER_TRUSS_RIGHT_100);
                    } else if ((colorSetting == Color.RED && sideSetting == Side.AIRSTRIP && propLocation == Prop.RIGHT) ||
                               (colorSetting == Color.BLUE && sideSetting == Side.BACKDROP && propLocation == Prop.RIGHT)) {
                        newDriveState(DriveState.STRAFE_OUT_FROM_UNDER_TRUSS_LEFT_105);
                    } else {
                        newDriveState(DriveState.TUCK_ARM_110);
                    }
                } else { // Stick around
                    robot.gyroDrive(-.2, 0.0, 0.03);
                }
                break; //}

            case STRAFE_OUT_FROM_UNDER_TRUSS_RIGHT_100: //{
                    telemetry.addData("Drive state", CurrentDriveState.name());
                    if (InitDriveState) { // Start starting
                        robot.resetDriveEncoders();
                        robot.gyroStrafe(.2, 0.0, 0.03);
                        InitDriveState = false;
                    }
                    if (gotStrafeDistance(6, true)) {
                        stopDrive();
                        newDriveState(DriveState.TUCK_ARM_110);
                    }
                    else {
                        robot.gyroStrafe(.2, 0.0, 0.03);
                    }
                    break; //}
            
            case STRAFE_OUT_FROM_UNDER_TRUSS_LEFT_105: //{
                    telemetry.addData("Drive state", CurrentDriveState.name());
                    if (InitDriveState) { // Start starting
                        robot.resetDriveEncoders();
                        robot.gyroStrafe(-.2, 0.0, 0.03);
                        InitDriveState = false;
                    }
                    if (gotStrafeDistance(-10, false)) {
                        stopDrive();
                        newDriveState(DriveState.TUCK_ARM_110);
                    }
                    else {
                        robot.gyroStrafe(-.2, 0.0, 0.03);
                    }
                    break; //}
                                           
            case TUCK_ARM_110: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState) { // Start starting
                    waitTime = DriveStateTime.milliseconds() + 100;
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_TUCK, robot.WRIST_TUCK, 1);
                    robot.lg_servo.setPosition(robot.LEFT_GRIP_CLOSED);
                    InitDriveState = false;
                }
                if (DriveStateTime.milliseconds() > waitTime) {
                    if (park || backPixel) {
                        newDriveState(DriveState.TURN_TO_FACE_BACKDROP_120);   
                    } else {
                        newDriveState(DriveState.END);
                    }
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_TUCK, robot.WRIST_TUCK, .02);
                }
                break; //}
                
            case TURN_TO_FACE_BACKDROP_120: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                // *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    if (colorSetting == Color.BLUE) {
                        turn(-.3, .3);
                    } else {
                        turn(.3, -.3);
                    }
                    InitDriveState = false;
                }
                if (colorSetting == Color.BLUE) {
                    if (gotAngle(85, false)) {  // Time to leave (Turned to heading, TRUE is clockwise)
                        stopDrive();
                        newDriveState(DriveState.DRIVE_TO_BACKBOARD_FAST_125);
                    }
                } else {    
                    if (gotAngle(-85, true)) {  // Time to leave (Turned to heading, TRUE is clockwise)
                        stopDrive();
                        newDriveState(DriveState.DRIVE_TO_BACKBOARD_FAST_125);
                    }
                }
                break; //}
                
            case DRIVE_TO_BACKBOARD_FAST_125: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                // *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(.5, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(.5, -90.0, 0.03);
                    }
                    InitDriveState = false;
                }
                // *** Reasons to leave state ***
                if (gotDistance(18, true)) {
                    stopDrive();
                    newDriveState(DriveState.DRIVE_TO_BACKBOARD_LINE_130);
                // *** Things to do every time the state is looped through ***
                } else {
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(.5, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(.5, -90.0, 0.03);
                    }
                }
                break; //}                
                
        case DRIVE_TO_BACKBOARD_LINE_130: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                // *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(.2, 90.0, 0.03);   // Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(.2, -90.0, 0.03);
                    }
                    InitDriveState = false;
                }
                // *** Reasons to leave state ***
                if (gotLine()) { // Line Detected    
                    stopDrive();
                    if (backPixel) {
                        newDriveState(DriveState.POSITION_ARM_FOR_BACKBOARD_140);
                    } else {
                        newDriveState(DriveState.STRAFE_TO_SIDE_OF_FIELD_200);
                    }
                // *** Things to do every time the state is looped through ***
                } else {
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(.2, 90.0, 0.03);   // Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(.2, -90.0, 0.03);
                    }
                }
                break; //}                
  
            case POSITION_ARM_FOR_BACKBOARD_140: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                // *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    waitTime = DriveStateTime.milliseconds() + 100; // Replace timeToWait with milliseconds
                    InitDriveState = false;
                }
                // *** Reasons to leave state ***
                if (DriveStateTime.milliseconds() > waitTime && robot.elbowWithinRange(robot.ELBOW_FRONT_DROP)) {  // timeToWait has elapsed
                    newDriveState(DriveState.STRAFE_TO_LINE_UP_PIXEL_150);
                // *** Things to do every time the state is looped through ***
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_FRONT_DROP, robot.WRIST_FRONT_DROP, 1);  // if wrist does move
                }
                break; //}
  
            case STRAFE_TO_LINE_UP_PIXEL_150:
                telemetry.addData("Drive state", CurrentDriveState.name());
                // *** Things to do first time in a state ***
                if (InitDriveState){
                    robot.resetDriveEncoders();
                    moveRight = true;
                    
                    if (colorSetting == Color.BLUE) {
                        distanceFromWall = robot.leftd_sensor.getDistance(DistanceUnit.INCH);
                        if (propLocation == Prop.LEFT) {targetDistanceFromWall = 19;}
                        else if (propLocation == Prop.CENTER) {targetDistanceFromWall = 25;}
                        else {targetDistanceFromWall = 29;}  // righh
                        if(targetDistanceFromWall < distanceFromWall) {moveRight = false;}
                        if(moveRight){robot.gyroStrafe(0.2, 90, 0.03);  }
                        else {robot.gyroStrafe(-0.2, 90, 0.03);}
                    }
                    if (colorSetting == Color.RED) {
                        distanceFromWall = robot.rightd_sensor.getDistance(DistanceUnit.INCH);
                        if (propLocation == Prop.LEFT) {targetDistanceFromWall = 30;}//35
                        else if (propLocation == Prop.CENTER) {targetDistanceFromWall = 27;}
                        else {targetDistanceFromWall = 23;}  // righh
                        
                        if(targetDistanceFromWall > distanceFromWall) {moveRight = false;}//
                        
                        if(moveRight){robot.gyroStrafe(0.2, -90, 0.03);  }
                        else {robot.gyroStrafe(-0.2, -90, 0.03);} 
                    }
                    /// red if go here   
                    InitDriveState = false;
                }
                // *** Reasons to leave state ***
                if (colorSetting == Color.BLUE && moveRight && robot.gotSideDistance(targetDistanceFromWall, false, false)) {
                    stopDrive();
                    newDriveState(DriveState.FINAL_APPROACH_BACKBOARD_160);
                } else if (colorSetting == Color.BLUE && !moveRight && robot.gotSideDistance(targetDistanceFromWall, false, true)){
                    stopDrive();
                    newDriveState(DriveState.FINAL_APPROACH_BACKBOARD_160);
                // red go here   
                } else if (colorSetting == Color.RED && moveRight && robot.gotSideDistance(targetDistanceFromWall, true, true)) {
                    stopDrive();
                    newDriveState(DriveState.FINAL_APPROACH_BACKBOARD_160);
                } else if (colorSetting == Color.RED && !moveRight && robot.gotSideDistance(targetDistanceFromWall, true, false)){
                    stopDrive();
                    newDriveState(DriveState.FINAL_APPROACH_BACKBOARD_160);
                
                
                } else {
                    if (colorSetting == Color.BLUE) {
                        if(moveRight){robot.gyroStrafe(0.2, 90, 0.03);  }
                        else {robot.gyroStrafe(-0.2, 90, 0.03);}
                    } // red go here
                    if (colorSetting == Color.RED) {
                        if(moveRight){robot.gyroStrafe(0.2, -90, 0.03);  }
                        else {robot.gyroStrafe(-0.2, -90, 0.03);}
                    } 
                
                }
                break; //}
  
            case FINAL_APPROACH_BACKBOARD_160: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                // *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(.2, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(.2, -90.0, 0.03);
                    }
                    InitDriveState = false;
                }
                // *** Reasons to leave state ***
                if (gotDistance(10, true)) {
                    stopDrive();
                    newDriveState(DriveState.DROP_PIXEL_170);
                // *** Things to do every time the state is looped through ***
                } else {
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(.2, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(.2, -90.0, 0.03);
                    }
                }
                break; //}
                
            case DROP_PIXEL_170: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState) {
                    waitTime = DriveStateTime.milliseconds() + 500;
                    robot.setRightGripperPosition(robot.RIGHT_GRIP_SLIGHT_OPEN);
                    InitDriveState = false;
                }
                if (DriveStateTime.milliseconds() > waitTime) {
                    newDriveState(DriveState.BACK_AWAY_FROM_BACKBOARD_180);
                } else {
                    
                }
                break; //}       
                
             case BACK_AWAY_FROM_BACKBOARD_180: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                // *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(-.2, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(-.2, -90.0, 0.03);
                    }
                    InitDriveState = false;
                }
                // *** Reasons to leave state ***
                if (gotDistance(-5, false)) {
                    stopDrive();
                    newDriveState(DriveState.TUCK_ARM_190);
                // *** Things to do every time the state is looped through ***
                } else {
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(-.2, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(-.2, -90.0, 0.03);
                    }
                }
                break; //}
  
                case TUCK_ARM_190: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (InitDriveState) { // Start starting
                    waitTime = DriveStateTime.milliseconds() + 100;
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_TUCK, robot.WRIST_TUCK, 1);
                    robot.rg_servo.setPosition(robot.RIGHT_GRIP_CLOSED);
                    InitDriveState = false;
                }
                if (DriveStateTime.milliseconds() > waitTime) {
                    if (park) {
                        newDriveState(DriveState.STRAFE_TO_SIDE_OF_FIELD_200);   
                    } else {
                        newDriveState(DriveState.END);
                    }
                } else {
                    armControl (robot.ELBOW_MAX_SPEED, robot.ELBOW_TUCK, robot.WRIST_TUCK, 1);
                }
                break; //}
  
            case STRAFE_TO_SIDE_OF_FIELD_200: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                // *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    if (colorSetting == Color.BLUE) {
                        robot.gyroStrafe(-.5, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroStrafe(.5, -90.0, 0.03);
                    }
                    InitDriveState = false;
                }
                // *** Reasons to leave state ***
                if (colorSetting == Color.BLUE &&  robot.gotSideDistance(2, false, true)) {
                    stopDrive();
                    newDriveState(DriveState.DRIVE_INTO_CORNER_210); 
                } else if (colorSetting == Color.RED && robot.gotSideDistance(2, true, true)) {
                    stopDrive();
                    newDriveState(DriveState.DRIVE_INTO_CORNER_210);
                } else {
                    if (colorSetting == Color.BLUE) {
                        robot.gyroStrafe(-.5, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroStrafe(.5, -90.0, 0.03);
                    }
                }
                break; //}

            case DRIVE_INTO_CORNER_210: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                // *** Things to do first time in a state ***
                if (InitDriveState)
                {
                    robot.resetDriveEncoders();
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(.5, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(.5, -90.0, 0.03);
                    }
                    InitDriveState = false;
                    
                }
                // *** Reasons to leave state ***
                if (gotDistance(20, true)) {
                    stopDrive();
                    newDriveState(DriveState.END);
                // *** Things to do every time the state is looped through ***
                } else {
                    if (colorSetting == Color.BLUE) {
                        robot.gyroDrive(.5, 90.0, 0.03);  // Positive speed is right, Recommended gain around 0.03
                    } else {
                        robot.gyroDrive(.5, -90.0, 0.03);
                    }
                }
                break; //}
                
            case END: //{
                telemetry.addData("Drive state", CurrentDriveState.name());
                if (true)
                {
                    stopDrive();
                }
                break; //}
        }

        // Show the elapsed game time and wheel power.
        robot.updatePersistentTelemetry();
        telemetryConfig();
        
        telemetry.addData("Disatnc", "is " + robot.rightd_sensor.getDistance(DistanceUnit.INCH));
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
    
    //TODO move to robot hardare
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