package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class robotHardware {
    private Limelight3A limelight;
    Testbench bench = new TestBench();
    public void calculateDistance(int pipeline) {
        double distance;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        YawPitchRollAngles orientation = bench.getOrientation();

        limelight.pipelineSwitch(pipeline);
        telemetry.addData("TA:", );
    }
}
