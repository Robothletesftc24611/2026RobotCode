package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Disabled
@Autonomous(name = "BLUE BACK - 6 RANDOM")
public class blueback6 extends OpMode {

    // Drive / subsystems
    private Follower follower;
    private DcMotor intake = null;
    private DcMotorEx shooter = null; // PIDF motor
    private Servo spindexer = null;
    private Servo scooper = null;
    private Servo door = null;
    public CRServo turretServo = null;
    private Limelight3A limelight = null;
    private VoltageSensor batteryVoltage = null;

    // Pedro pathing
    private int pathState = 0;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(270));
    private final Pose pickup1 = new Pose(-12, 27, Math.toRadians(0));
    private final Pose pickup1a = new Pose(-20, 27, Math.toRadians(0));

    private final Pose pickup1b = new Pose(-25,27,Math.toRadians(0));
    private final Pose pickup1c = new Pose(-35, 27, Math.toRadians(0));
    private final Pose park = new Pose(-34, 68, Math.toRadians(0));
    private PathChain goto1, pick1a, pick1b, pick1c, shoot2, last ;

    // Shooting / spindexer
    private final double[] spindexerPositions = {-0.1, 0.4, 0.85};
    private int shootState = 0;
    private int ballsShot = 0;
    private boolean shootingDone = false;

    // shooter PIDF constants (copied from teleop)
    private static final double NOMINAL_VOLTAGE = 11.38;
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 22.5; // base feedforward constant

    // turret / limelight tuning
    private final double LIMELIGHT_OFFSET = -1.0;
    private final double DEADZONE = 1.0;
    private final double MAX_POWER = 1.0;

    // timers
    private ElapsedTime actionTimer = new ElapsedTime();
    private ElapsedTime opmodeTimer = new ElapsedTime();

    @Override
    public void init() {
        // hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        spindexer = hardwareMap.get(Servo.class, "spindexer");
        scooper = hardwareMap.get(Servo.class, "scooper");
        door = hardwareMap.get(Servo.class, "door");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // voltage sensor for F compensation
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        // servo defaults (match teleop)
        scooper.setDirection(Servo.Direction.REVERSE);
        spindexer.setPosition(spindexerPositions[0]);
        scooper.setPosition(0.0);
        door.setPosition(0.2); // safe default

        // shooter setup
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        applyShooterPIDFCompensated();

        // limelight
        limelight.pipelineSwitch(2);
        limelight.start();

        // path follower
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        actionTimer.reset();
        opmodeTimer.reset();

        telemetry.addData("Status", "Initialized (turret+PIDF integrated)");
        telemetry.update();
    }

    private void applyShooterPIDFCompensated() {
        double voltage = batteryVoltage.getVoltage();
        if (voltage <= 0) voltage = NOMINAL_VOLTAGE; // safety
        double compensatedF = kF * (NOMINAL_VOLTAGE / voltage);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, compensatedF));
    }

    private void buildPaths() {
        goto1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1.getHeading())
                .addParametricCallback(0.01, () -> limelight.stop())
                .addParametricCallback(0.2, () -> intake.setPower(-0.6))
                .addParametricCallback(0.2, () -> door.setPosition(0.0))
                .build();
        pick1a = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, pickup1a))
                .setLinearHeadingInterpolation(pickup1.getHeading(), pickup1a.getHeading())
                .addParametricCallback(0.9, () -> spindexer.setPosition(0.5))
                .build();
        pick1b = follower.pathBuilder()
                .addPath(new BezierLine(pickup1a, pickup1b))
                .setLinearHeadingInterpolation(pickup1a.getHeading(), pickup1b.getHeading())
                .addParametricCallback(0.9, () -> spindexer.setPosition(1.0))
                .build();
        pick1c = follower.pathBuilder()
                .addPath(new BezierLine(pickup1b, pickup1c))
                .setLinearHeadingInterpolation(pickup1b.getHeading(), pickup1c.getHeading())
                .addParametricCallback(0.8, () -> spindexer.setPosition(0.1))
                .addParametricCallback(0.8, () -> door.setPosition(0.2))
                .addParametricCallback(0.8, () -> intake.setPower(0.0))
                .build();
        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1c, startPose))
                .setLinearHeadingInterpolation(pickup1c.getHeading(), startPose.getHeading())
                .addParametricCallback(0.7, () -> limelight.start())
                .build();
        last = follower.pathBuilder()
                .addPath(new BezierLine(startPose, park))
                .setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading())
                .build();
    }

    @Override
    public void init_loop() {
        // nothing required here
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        actionTimer.reset();
        pathState = 0;
        shootingDone = false;
        shootState = 0;
        ballsShot = 0;
    }

    @Override
    public void loop() {
        // always run follower update
        follower.update();

        // path state machine
        autonomousPathUpdate();

        // update turret tracking continuously (so it can track while shooting)
        turretAutoTrack();

        // shooting state machine runs independently but is triggered from path logic
        shootThreeUpdate(); // uses shooter.setVelocity(...) and spindexer/scooper timing

        // telemetry
        telemetry.addData("path state", pathState);
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("pose x", follower.getPose().getX());
        telemetry.addData("pose y", follower.getPose().getY());
        telemetry.addData("pose heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("shootState", shootState);
        telemetry.addData("ballsShot", ballsShot);
        telemetry.addData("shooter velocity (target)", getExpectedShooterVelocityForRange());
        telemetry.addData("shooter actual velocity", shooter.getVelocity());
        telemetry.addData("actionTimer", actionTimer.seconds());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // start following preload score path
                shootState = 0;
                ballsShot = 0;
                shootingDone = false;
                actionTimer.reset();
                pathState = 1;
                break;

            case 1:
                if (shootingDone){
                    follower.followPath(goto1);
                    follower.setMaxPower(0.4);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    follower.followPath(pick1a);
                    follower.setMaxPower(0.4);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    opmodeTimer.reset();
                    pathState = 4;
                }
                break;
            case 4:
                if (opmodeTimer.seconds() > 0.5){
                    follower.followPath(pick1b);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    opmodeTimer.reset();
                    pathState = 6;
                }
                break;
            case 6:
                if (opmodeTimer.seconds() > 0.5){
                    follower.followPath(pick1c);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.setMaxPower(1.0);
                    follower.followPath(shoot2);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    shootState = 0;
                    ballsShot = 0;
                    shootingDone = false;
                    actionTimer.reset();
                    pathState = 9;
                }
                break;
            case 9:
                if (shootingDone){
                    follower.followPath(last);
                    pathState = 10;
                }
                break;
            case 10:
                break;
        }
    }

    // Helper to pick target velocity based on limelight area (Ta)
    private double getExpectedShooterVelocityForRange() {
        // fallback default
        double defaultVelocity = 1325;
        try {
            LLResult llResult = limelight.getLatestResult();
            double ta = llResult.getTa(); // target area
            // adopt simple linear mapping similar to TeleOp example:
            double distance = (-33.74145 * ta) + 194.923;
            if (distance > 170) return 1325;
            else return 1325;
        } catch (Exception e) {
            return defaultVelocity;
        }
    }

    // Shooting FSM that uses velocity control and the spindexer/scooper servos
    public void shootThreeUpdate() {
        switch (shootState) {
            case 0:
                // start spin-up
                applyShooterPIDFCompensated(); // update for voltage changes just before spinup
                shooter.setVelocity(getExpectedShooterVelocityForRange());
                ballsShot = 0;
                spindexer.setPosition(spindexerPositions[ballsShot]);
                actionTimer.reset();
                shootState = 10;
                break;

            case 10:
                // give motor time to ramp
                if (actionTimer.seconds() > 2.0) {
                    actionTimer.reset();
                    shootState = 1;
                }
                break;

            case 1:
                // index then scoop
                // ensure shooter is near target (optional safety threshold)
                // Skipping strict velocity check to avoid hangups; rely on delay
                if (actionTimer.seconds() > 1) {
                    scooper.setPosition(0.5); // push ball into flywheel
                    actionTimer.reset();
                    shootState = 2;
                }
                break;

            case 2:
                // retract scooper and advance spindexer
                if (actionTimer.seconds() > 0.5) {
                    scooper.setPosition(0.0);
                    ballsShot++;

                    if (ballsShot < spindexerPositions.length) {
                        spindexer.setPosition(spindexerPositions[ballsShot]);
                        actionTimer.reset();
                        shootState = 1; // index next ball
                    } else {
                        // finished shooting sequence
                        shooter.setVelocity(0);
                        spindexer.setPosition(spindexerPositions[0]);
                        shootingDone = true;
                        shootState = 3;
                    }
                }
                break;

            case 3:
                // finished - idle state
                // nothing else to do; remain here
                break;

            default:
                break;
        }
    }

    // Turret auto-tracking using Limelight (same logic as TeleOp but without manual override)
    private void turretAutoTrack() {
        try {
            LLResult llResult = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                double tx = llResult.getTx();
                double correctedTx = tx - LIMELIGHT_OFFSET;

                if (Math.abs(correctedTx) > DEADZONE) {
                    double power = -correctedTx / 30.0;
                    power = Range.clip(power, -MAX_POWER, MAX_POWER);
                    turretServo.setPower(power);
                } else {
                    turretServo.setPower(0.0);
                }
            } else {
                turretServo.setPower(0.0); // hold if no tag
            }
        } catch (Exception e) {
            // stop turret if limelight gives issues
            turretServo.setPower(0.0);
        }
    }

    @Override
    public void stop() {
        // shutdown safely
        shooter.setVelocity(0);
        turretServo.setPower(0.0);
        scooper.setPosition(0.0);
        spindexer.setPosition(spindexerPositions[0]);
    }
}
