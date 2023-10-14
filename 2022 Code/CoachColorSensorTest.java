package org.firstinspires.ftc.teamcode;
 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Coach: Color Sensor Test", group="Coach")
public class CoachColorSensorTest extends LinearOpMode {
    // Define a variable for our color sensor
    ColorRangeSensor Color_Sensor;
    
    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        Color_Sensor = hardwareMap.get(ColorRangeSensor.class, "Color_Sensor");
        
        // Wait for the Play button to be pressed
        waitForStart();
 
        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", Color_Sensor.red());
            telemetry.addData("Green", Color_Sensor.green());
            telemetry.addData("Blue", Color_Sensor.blue());
            telemetry.addData("Distance", Color_Sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            
        }
    }
}