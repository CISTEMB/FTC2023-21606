package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.GyroSensor;

public class RobotHardwareAJ
{
    /* Public OpMode members. */
    public DcMotor leftmotor;
    public DcMotor rightmotor;
    public Servo servoArm;
    
    //Instantiate sensors
    //public GyroSensor gyroSensor;

    /* local OpMode members. */
    HardwareMap hardwareMap;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {

        // Define and Initialize Motors and servos
        leftmotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightmotor = hardwareMap.get(DcMotor.class, "rightmotor");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        
        //Define sensor
        //gryoSensor=hardewareMap.get(GyroSensor.class, "gyroSensor");
        
        // Set all motors to zero power
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        
        // set mode
        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // can also use RunMode.RUN_TO_POSITION
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // SET MOTOR POWER BEHAVIOR
        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // also ZeroPowerBehavior.float
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //Servo position
        servoArm.setPosition(0);
        
        //Calibrate sensors
        //gyroSensor.calibrate();
    }
 }

