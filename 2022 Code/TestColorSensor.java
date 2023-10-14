package org.firstinspires.ftc.teamcode;
 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 
@TeleOp(name="Test: Color Sensor", group="Test")
public class TestColorSensor extends LinearOpMode {
    // Define a variable for our color sensor
    ColorRangeSensor color;
    
    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorRangeSensor.class, "Color_Sensor");
        
        double redGain = 2;
        double blueGain = 1;
        double greenGain = 1;
        
        // Wait for the Play button to be pressed
        waitForStart();
 
        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red()*redGain);
            telemetry.addData("Green", color.green()*greenGain);
            telemetry.addData("Blue", color.blue()*blueGain);
            telemetry.addData("Distance", color.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
