package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LimeLightTest extends OpMode{
    Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

    public void init() {
        // Limelight config
        limelight.setPollRateHz(100); // How many times (per second) we get data
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void loop() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Target Area:", result.getTa());
    }
}
