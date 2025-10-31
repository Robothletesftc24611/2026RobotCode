package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LimeLightTest extends OpMode{
    private Limelight3A limelight;

    private double distance;

    public void init() {
        // Limelight config
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); //Test with red goal
    }

    public void start(){
        limelight.start();
    }


    public void loop() {
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            telemetry.addData("Target Area:", llresult.getTa());
        }
    }
}
