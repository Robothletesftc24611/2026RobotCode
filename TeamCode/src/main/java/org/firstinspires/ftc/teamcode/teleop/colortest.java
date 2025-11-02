package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@TeleOp
public class colortest extends LinearOpMode {
    ColorSensor colorSensor;

    public void runOpMode(){
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();

        while (opModeIsActive()) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            String detectedColor = "Nothing";

            // Detect green
            if ((green >= 105) && (blue <= 130) && (red <= 65)){
                detectedColor = "Green";
            }
            // Detect Purple
            else if ((red >= 70) && (blue >= 110)){
                detectedColor = "Purple";
            }

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();
        }
    }

}
