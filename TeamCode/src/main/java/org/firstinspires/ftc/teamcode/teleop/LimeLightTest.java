package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="Limelight Shooter Tuning", group="Shooter")
public class LimeLightTest extends OpMode {

    private Limelight3A limelight;
    private DcMotorEx shooter;
    private VoltageSensor batteryVoltage;

    // --- PIDF values ---
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double kP = 0.0006;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 19; // Tune F first

    private double targetVelocity = 0;
    private double distance = 0;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        // Set PIDF only one time throughout the code (with voltage compensation)
        double compensatedF = kF * (NOMINAL_VOLTAGE / batteryVoltage.getVoltage());
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, compensatedF));

        telemetry.addLine("Shooter PIDF Initialized");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        // check limelight target
        if (result != null && result.isValid()) {
            distance = getDistanceFromTag(result.getTa());
            targetVelocity = getVelocityFromDistance(distance);
        }

        // give the shooter a target velocity to try to reach
        shooter.setVelocity(1600); // desired speed, the speed that we want the motor to run at

        telemetry.addData("Limelight TA", result != null ? result.getTa() : "none");
        telemetry.addData("Distance (cm)", distance);
        telemetry.addData("Target Velocity", 1600);
        telemetry.addData("Actual Velocity", shooter.getVelocity());
        telemetry.addData("Error", 1600 - shooter.getVelocity());
        telemetry.update();
    }

    // Approx conversion TA → distance
    private double getDistanceFromTag(double ta) {
        return (-33.74145 * ta) + 194.923;
    }

    // Map distance → shooter RPM
    private double getVelocityFromDistance(double distance) {
        return 1800; // constant for now, testing purposes
    }
}
