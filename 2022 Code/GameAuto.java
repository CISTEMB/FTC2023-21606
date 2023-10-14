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

//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
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
@Autonomous(name="Game: Autonomous", group="Game", preselectTeleOp="Game: TeleOp Drive - LFA")

public class GameAuto extends LinearOpMode {
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
    private ColorRangeSensor Color_Sensor;


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
        Color_Sensor = hardwareMap.get(ColorRangeSensor.class, "Color_Sensor");
        
        
        // ****** Set-Up Gripper ****** //
        final double wristUp = 1;   //The postition the wrist is at when it is up
        final double wristFlat = 0.525;
        Wrist_Servo.setPosition(wristUp);
        Lg_Servo.setPosition(-1);
        Rg_Servo.setPosition(1);
        
        // ****** Drive Motor Setup ****** //
        //Setting up the variables for drive
        //double pc=0.5;  //power coeficcient
        //double sl=0.65; //speed limit
        int motorPlace = 0;   // How far the motor goes
        //Set variables for motor speed
        double lfc=0;   //Left front motor speed
        double rfc=0;   //Right front motor speed
        double lbc=0;   //Left back motor speed
        double rbc=0;   //Right back motor speed
        
        //Set motors to correct direction
        double square1 = 21.25; // How far the robot moves when it hits the starting position
        
        Lf_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Rf_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Rb_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        Lf_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lf_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lf_Motor.setTargetPosition(motorPlace);
        Lf_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        Lb_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lb_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lb_Motor.setTargetPosition(motorPlace);
        Lb_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        Rf_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rf_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rf_Motor.setTargetPosition(motorPlace);
        Rf_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        Rb_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rb_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rb_Motor.setTargetPosition(motorPlace);
        Rb_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       
        // ****** Set-Up Telemtry ****** //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            
            //drive to cone
            straightDrive(square1, 0.3, 6); 
            
            //read singal
            int signal = readSignal();
            
            //drive to center of the square
            straightDrive(8, 0.5, 3);
            
            //if move right
            if (signal == 3) {
                //staraph right
                strafeDrive (24,0.5,5);
            } else if (signal == 1){
                //staraph left
                strafeDrive (-24,0.5,5);
            } else {
                straightDrive(8, 0.5, 3);
                straightDrive(-8, 0.5, 3);
            }
            
            //Wrist_Servo.setPosition(wristFlat);
            sleep(30000);
        }
    }
    
    // METHOD 2R DRIVE
    public void straightDrive (double distance, double power, double runTime) {
        
        //  1000 counts equals 18.25 inches
        int counts = (int) (distance*1000/18.5);
        
        double stTime = time;
        
        int LfCounts = Lf_Motor.getCurrentPosition()+ counts;
        int RfCounts = Rf_Motor.getCurrentPosition()+ counts;
        int LbCounts = Lb_Motor.getCurrentPosition()+ counts;
        int RbCounts = Rb_Motor.getCurrentPosition()+ counts;
        
        Lf_Motor.setTargetPosition(LfCounts);
        Lb_Motor.setTargetPosition(LbCounts);
        Rf_Motor.setTargetPosition(RfCounts);
        Rb_Motor.setTargetPosition(RbCounts);
            
        Lf_Motor.setPower(power);
        Rf_Motor.setPower(power);
        Lb_Motor.setPower(power);
        Rb_Motor.setPower(power);
            
        while ((Lf_Motor.isBusy() || Rf_Motor.isBusy()) && opModeIsActive()){
            telemetry.addData("method","straightDrive" );
            telemetry.addData("T distance",distance);
            telemetry.addData("T power",power);
            telemetry.addData("T counts", counts);
            telemetry.addData("T LfCounts", LfCounts);
            telemetry.addData("T RfCounts", RfCounts);
            telemetry.addData("T LbCounts", LbCounts);
            telemetry.addData("T RbCounts", RbCounts);
            telemetry.addData("Lf counts",Lf_Motor.getCurrentPosition() );
            telemetry.addData("Rf counts",Rf_Motor.getCurrentPosition() );
            telemetry.addData("Lb counts",Lb_Motor.getCurrentPosition() );
            telemetry.addData("Rb counts",Rb_Motor.getCurrentPosition() );
            telemetry.update();
            
            if (time - stTime > runTime){
                Lf_Motor.setTargetPosition(Lf_Motor.getCurrentPosition());
                Rf_Motor.setTargetPosition(Rf_Motor.getCurrentPosition());
                Lb_Motor.setTargetPosition(Lb_Motor.getCurrentPosition());
                Rb_Motor.setTargetPosition(Rb_Motor.getCurrentPosition());
            }
            
            sleep(100);
        }
            
        Lf_Motor.setPower(0);
        Rf_Motor.setPower(0);
        Lb_Motor.setPower(0);
        Rb_Motor.setPower(0); 
    }
    
    
    
    // METHOD 2R STRAFE
    public void strafeDrive (double distance, double power, double runTime) {
        
        //  1000 counts equals 17.25 inches
        int counts = (int) (distance*1000/17.5);
        
        double stTime = time;
        
        int LfCounts = Lf_Motor.getCurrentPosition()+ counts;
        int RfCounts = Rf_Motor.getCurrentPosition()- counts;
        int LbCounts = Lb_Motor.getCurrentPosition()- counts;
        int RbCounts = Rb_Motor.getCurrentPosition()+ counts;
        
        Lf_Motor.setTargetPosition(LfCounts);
        Lb_Motor.setTargetPosition(LbCounts);
        Rf_Motor.setTargetPosition(RfCounts);
        Rb_Motor.setTargetPosition(RbCounts);
            
        Lf_Motor.setPower(power);
        Rf_Motor.setPower(power);
        Lb_Motor.setPower(power);
        Rb_Motor.setPower(power);
            
        while ((Lf_Motor.isBusy() || Rf_Motor.isBusy()) && opModeIsActive()){
            telemetry.addData("method","strafeDrive" );
            telemetry.addData("T distance",distance);
            telemetry.addData("T power",power);
            telemetry.addData("T counts", counts);
            telemetry.addData("T LfCounts", LfCounts);
            telemetry.addData("T RfCounts", RfCounts);
            telemetry.addData("T LbCounts", LbCounts);
            telemetry.addData("T RbCounts", RbCounts);
            telemetry.addData("Lf counts",Lf_Motor.getCurrentPosition() );
            telemetry.addData("Rf counts",Rf_Motor.getCurrentPosition() );
            telemetry.addData("Lb counts",Lb_Motor.getCurrentPosition() );
            telemetry.addData("Rb counts",Rb_Motor.getCurrentPosition() );
            telemetry.update();
            
            if (time - stTime > runTime){
                Lf_Motor.setTargetPosition(Lf_Motor.getCurrentPosition());
                Rf_Motor.setTargetPosition(Rf_Motor.getCurrentPosition());
                Lb_Motor.setTargetPosition(Lb_Motor.getCurrentPosition());
                Rb_Motor.setTargetPosition(Rb_Motor.getCurrentPosition());
            }
            
            sleep(100);
     
        }
            
        Lf_Motor.setPower(0);
        Rf_Motor.setPower(0);
        Lb_Motor.setPower(0);
        Rb_Motor.setPower(0); 
    }
    

    //METHOD 2R SIGNAL
    public int readSignal(){
        double stTime = time;
        double redGain = 2;
        double greenGain = 1;
        double blueGain = 1;
        
        while ((time - stTime < 5) && opModeIsActive()) {
            
            telemetry.addData("Red", Color_Sensor.red()*redGain);
            telemetry.addData("Green", Color_Sensor.green()*greenGain);
            telemetry.addData("Blue", Color_Sensor.blue()*blueGain);
            telemetry.addData("Distance", Color_Sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        double red = Color_Sensor.red()*redGain;
        double green = Color_Sensor.green()*greenGain;
        double blue = Color_Sensor.blue()*blueGain;
        
        if((red > blue) && (red > green)) {
            return 1;
            
        } else if ((blue > red) && (blue > green)) {
            return 3;
        } else {
            return 2;
        }
        
        
    }
    
}
