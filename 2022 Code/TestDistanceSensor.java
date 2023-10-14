package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name="Test: Distance Sensor", group="Test")
public class TestDistanceSensor extends LinearOpMode {
    // Define a variable for the distance sensor
    private DistanceSensor distance_sensor;

    @Override
    public void runOpMode() {
        // Get the distance sensor from hardwareMap
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        
        // Wait for the Play button to be pressed
        waitForStart();
 
        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Distance", distance_sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}