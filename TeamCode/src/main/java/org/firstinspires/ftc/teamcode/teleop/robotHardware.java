package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;


import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class robotHardware {
    private Limelight3A limelight;
    Testbench bench = new TestBench();
    public void calculateDistance(int pipeline) {
        double distance;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);

        YawPitchRollAngles orientation = bench.getOrientation();

        limelight.pipelineSwitch(pipeline);
        telemetry.addData("TA:",);
    }
    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop(){

    }
}
