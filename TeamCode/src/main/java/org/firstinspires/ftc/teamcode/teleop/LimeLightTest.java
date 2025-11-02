package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp(name="Limelight Distance", group="OpMode")
public class LimeLightTest extends OpMode{
    private Limelight3A limelight;

    private DcMotorEx Shooter;
    private VoltageSensor batteryVoltageSensor;

    private double distance;
    private double targetVelocity;

    private static final double NOMINAL_VOLTAGE = 12.79; //write voltage that we are testing at
    private static final double kP = 0.0; //add P for stability
    private static final double kI = 0.0; //dont touch
    private static final double kD = 0.0; //dont touch
    private static final double kF = 15; // Tune F first

    public void init() {
        // Limelight config
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); //Test with red goal

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void start(){
        limelight.start();
    }


    public void loop() {
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            distance = getDistanceFromTag(llresult.getTa());
            targetVelocity = getVelocityFromDistance(distance);

            Shooter.setVelocity(targetVelocity);

            telemetry.addData("Target Area:", llresult.getTa());
            telemetry.addData("distance from tag(cm)", distance);
            telemetry.addData("Shooter velocity", Shooter.getVelocity());
            telemetry.addData("Target Velocity (ticks/sec)", targetVelocity);
        }else{
            telemetry.addLine("No valid Limelight target detected");
        }
        telemetry.update();
    }

    public double getDistanceFromTag(double ta) {
        double distance = ((-33.74145 * ta) + 194.923);
        return distance;
    }

    public double getVelocityFromDistance(double distance){
        //Linear model, tune based on shooter testing
        return 10*distance + 1000 ; //Ticks per second
    }
}
