package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@TeleOp(name="WheelTest")

public class WheelTest extends OpMode {
    private RobotHardware robot = new RobotHardware(this);
    
    @Override
    public void init() {
    }
    
    @Override
    public void init_loop() {
    }
    
    @Override
    public void start() {
    }
    
    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
        }
        if (gamepad1.b) {
            robot.setDrivePower(0, 0, 0, 0);
        }
        if (gamepad1.x) {
            robot.setDrivePower(0.25, 0.25, 0.25, 0.25);
            robot.setDrivePosition(robot.lf_motor.getCurrentPosition()+100, 
                                   robot.rf_motor.getCurrentPosition()+100, 
                                   robot.lb_motor.getCurrentPosition()+100, 
                                   robot.rb_motor.getCurrentPosition()+100
            );
            robot.setDriveMode(RunMode.RUN_TO_POSITION);
        }
    }
}