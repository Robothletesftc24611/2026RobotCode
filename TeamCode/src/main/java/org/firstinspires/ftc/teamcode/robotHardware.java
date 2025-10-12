package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private TestBench bench;  // assuming you have a TestBench class

    // Constructor
    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.bench = new TestBench();

        // Initialize the Limelight camera
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void calculateDistance(int pipeline) {
        // Switch to the correct Limelight pipeline
        limelight.pipelineSwitch(pipeline);

        // Get latest result from Limelight
        LLResult result = limelight.getLatestResult();

        // Get robot orientation (if TestBench provides it)
        YawPitchRollAngles orientation = bench.getOrientation();

        if (result != null && result.isValid()) {
            double targetArea = result.getTa();  // Example: target area
            double tx = result.getTx();          // Horizontal offset
            double ty = result.getTy();          // Vertical offset

            telemetry.addData("Target Area", targetArea);
            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
        } else {
            telemetry.addLine("No valid Limelight result.");
        }

        telemetry.update();
    }
}
