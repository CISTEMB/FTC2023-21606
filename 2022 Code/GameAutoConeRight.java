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
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


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
@Autonomous(name="Game: Autonomous Cone Right Side", group="Game", preselectTeleOp="Game: TeleOp Drive - LFA")

public class GameAutoConeRight extends LinearOpMode {
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
    
    //gyro stuff
    BNO055IMU               IMU;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;

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
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        
        // ****** Set-Up Gripper ****** //
        final double wristUp = 1;   //The postition the wrist is at when it is up
        final double wristFlat = 0.525;
        final double wristMid = 0.084;
        Wrist_Servo.setPosition(wristUp);
        Lg_Servo.setPosition(-1);
        Rg_Servo.setPosition(1);
        int armPlaceMid = 360;
        
        // ****** Servo Telemtry ****** //
        telemetry.addData("Status", "Updating servos");
        telemetry.update();
        sleep(2000);  //wait for servos to stop moving
        telemetry.addData("Status", "Servos done");
        telemetry.update();
        
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
        double square1 = 22.25; // How far the robot moves when it hits the starting position
        
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
       
        //Setting up the arm motors
        Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Motor.setTargetPosition(0);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //set up IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
       
        // ****** Gyro Telemtry and Calibration****** //
        telemetry.addData("Status", "Calibrating Gyro");
        telemetry.update();
        IMU.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !IMU.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "Gyro calibrated");
        telemetry.addData("imu calib status", IMU.getCalibrationStatus().toString());
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            
            /*
            gyroTurn(90,0.3,20);
            sleep(500);
            gyroTurn(90,0.1,20);
            gyroTurn(90,0.1,20);
            sleep(600000);
            */
            
            //drive to cone
            straightDrive(square1, 0.3, 6); 
            
            //read singal
            int signal = readSignal();
            
            //drive to push signal out of the way
            straightDrive(16, 0.3, 3);
            
            //back up to the ceter of the square
            straightDrive(-8, 0.3, 3);
            
            //place arm in position
            Arm_Motor.setPower(0.5);
            Arm_Motor.setTargetPosition(armPlaceMid);
            sleep(1500);
            
            //place wrist in position
            Wrist_Servo.setPosition(wristMid);
            
            //drive to line up to the pole
            strafeDrive(-16, 0.3, 3);
            
            //drive till hits pole
            straightDrive(6, 0.05, 5);
            sleep (100);
            
            // back up from pole
            straightDrive(-2, 0.1, 5);
            sleep(100);
            
            //find pole with color sensor
            strafeDrivePole (7,0.1,5,4.2);
            sleep(100);
            
            // back up so cone drops on pole
            straightDrive(-1.75, 0.1, 5);
            sleep(100);
            
            //drop cone
            Lg_Servo.setPosition(1);
            Rg_Servo.setPosition(0);
            sleep(100);
            
            //recenter
            strafeDrive(12.5, 0.3, 5);
            
            //close wrist
            Lg_Servo.setPosition(0);
            Rg_Servo.setPosition(1);
            
            //tuck wrist
            Wrist_Servo.setPosition(1);
            
            //lower arm
            Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_Motor.setPower(-0.2);
            sleep(1000);
            
            //if move right
            if (signal == 3) {
                //staraph right
                strafeDrive (24,0.5,5);
            } else if (signal == 1){
                //staraph left
                strafeDrive (-24,0.5,5);
            } else {
                //straightDrive(8, 0.5, 3);
                //straightDrive(-8, 0.5, 3);
            }
            
            //turn around
            gyroTurn(-180,0.3,20);
            gyroTurn(-180,0.1,20);
            
            //putting wrist in starting position
            //Wrist_Servo.setPosition(wrist);
          
            
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
        
        while ((time - stTime < 1) && opModeIsActive()) {
            
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
    
    // METHOD 2R STRAFE POLE
    public void strafeDrivePole (double distance, double power, double runTime, double poleDistance) {
        
        //  1000 counts equals 17.25 inches
        int counts = (int) (distance*1000/17.5);
        boolean poleFound = false;
        double sensorDistance = 6;
        
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
            
        while ((Lf_Motor.isBusy() || Rf_Motor.isBusy()) && !poleFound && opModeIsActive()){
            sensorDistance = Color_Sensor.getDistance(DistanceUnit.INCH);
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
            telemetry.addData("Distace", sensorDistance);
            telemetry.update();
            
            if(sensorDistance < poleDistance){
                poleFound=true;
            }
            
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
    
    // METHOD 2R GYRO TURN
    public void gyroTurn (double targetAngle, double power, double runTime) {
        
        //  1000 counts equals 17.25 inches
        double distance = 56;  // Enough for 360
        int counts = (int) (distance*1000/17.5);
        double currentAngle;
        boolean counterClockwise = true;
        boolean angleFound = false;
        double stTime = time;
        
        //  Reverse motion if clockwise
        if (getAngle() > targetAngle ){
            counterClockwise = false; // going clockwise
            counts = counts*(-1);
        }
        
        int LfCounts = Lf_Motor.getCurrentPosition()- counts;
        int RfCounts = Rf_Motor.getCurrentPosition()+ counts;
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
            
        while ((Lf_Motor.isBusy() || Rf_Motor.isBusy()) && !angleFound && opModeIsActive()){
            currentAngle = getAngle();
            telemetry.addData("method","gyroTurn" );
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
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle",targetAngle );
            telemetry.update();
            
            if(counterClockwise){
                if(currentAngle > targetAngle){
                    angleFound = true;
                }
            } else{
                if(currentAngle < targetAngle){
                    angleFound = true;
                }
            }
            
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
    
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    
}
