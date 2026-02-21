package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "red 12", group = "COMPETITION")
public class red12closev3 extends OpMode {

    // === PATHING ===
    private Follower follower;
    private int pathState;

    // === SHOOTER SYSTEM ===
    private DcMotorEx shooter1, shooter2;
    private DcMotorEx intake;
    private Servo gate;

    private final double shooter_kP = 15;
    private final double shooter_kF = 12;
    private final double nominalVoltage = 12.7;
    private final double closeShotVelocity = 1450;

    private static final double GATE_CLOSED = 0.6;
    private static final double GATE_OPEN = 0.85;

    // === TURRET ===
    private DcMotorEx turret;
    private static final int TURRET_START_TICKS = 210; // example: 180 degrees if 4.5 ticks/deg

    // === SHOOT FSM ===
    private enum ShootState { IDLE, SPINUP, FIRE, DONE }
    private ShootState shootState = ShootState.IDLE;
    private long fireStartTime = 0;

    // === POSES ===
    private final Pose startPose   = new Pose(120.07, 128.46, Math.toRadians(220));
    private final Pose shootPose   = new Pose(87, 83, Math.toRadians(180));
    private final Pose finaltwo    = new Pose(125.1, 55, Math.toRadians(180));
    private final Pose control1    = new Pose(75, 52, Math.toRadians(180));
    private final Pose gatePose    = new Pose(127, 69, Math.toRadians(180));
    private final Pose control2    = new Pose(102.92, 68, Math.toRadians(180));
    private final Pose finalone    = new Pose(125, 84, Math.toRadians(180));
    private final Pose finalthree  = new Pose(130.14, 35.8, Math.toRadians(180));
    private final Pose control3    = new Pose(62, 26, Math.toRadians(180));

    private final Pose control4 = new Pose(88, 63, Math.toRadians(180));
    private final Pose finalPose   = new Pose(112.17, 68, Math.toRadians(180));

    private PathChain preload, pick2, opengate, shoot2, pick1, shoot1, pick3, shoot3, park;

    // === BUILD PATHS (your original structure preserved) ===
    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pick2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control1, finaltwo))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finaltwo.getHeading())
                .addParametricCallback(0.1, () -> intake.setPower(-1.0))
                .build();

        opengate = follower.pathBuilder()
                .addPath(new BezierCurve(finaltwo, control2, gatePose))
                .setLinearHeadingInterpolation(finaltwo.getHeading(), gatePose.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, control4, shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
                .build();

        pick1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finalone))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalone.getHeading())
                .addParametricCallback(0.1, () -> intake.setPower(-1.0))
                .build();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(finalone, shootPose))
                .setLinearHeadingInterpolation(finalone.getHeading(), shootPose.getHeading())
                .build();

        pick3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, control3, finalthree))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalthree.getHeading())
                .addParametricCallback(0.2, () -> intake.setPower(-1.0))
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(finalthree, shootPose))
                .setLinearHeadingInterpolation(finalthree.getHeading(), shootPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalPose.getHeading())
                .build();
    }

    // === SHOOTER HELPERS ===
    private double getCompensatedF() {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        return shooter_kF * (nominalVoltage / voltage);
    }

    private void setShooterVelocity(double target) {
        double f = getCompensatedF();
        shooter1.setVelocityPIDFCoefficients(shooter_kP, 0, 0, f);
        shooter2.setVelocityPIDFCoefficients(shooter_kP, 0, 0, f);
        shooter1.setVelocity(target);
        shooter2.setVelocity(target);
    }

    private void stopShooter() {
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }

    // === SHOOT FSM ===
    private void updateShootFSM() {
        switch (shootState) {

            case IDLE:
                break;

            case SPINUP:
                // shooter spins while robot drives
                break;

            case FIRE:
                long elapsed = System.currentTimeMillis() - fireStartTime;

                gate.setPosition(GATE_OPEN);

                if (elapsed > 250) intake.setPower(-1.0);
                else intake.setPower(0);

                if (elapsed > 1500) shootState = ShootState.DONE;
                break;

            case DONE:
                stopShooter();
                gate.setPosition(GATE_CLOSED);
                intake.setPower(0);
                shootState = ShootState.IDLE;
                break;
        }
    }

    // === AUTONOMOUS STATE MACHINE ===
    public void autonomousPathUpdate() {
        updateShootFSM();

        switch (pathState) {

            case -1: // TURRET INITIALIZATION
                if (!turret.isBusy()) {
                    turret.setPower(0);
                    pathState = 0;
                }
                break;

            case 0: // PRELOAD → SHOOT
                setShooterVelocity(closeShotVelocity);
                shootState = ShootState.SPINUP;
                follower.setMaxPower(0.7);

                follower.followPath(preload);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    shootState = ShootState.FIRE;
                    fireStartTime = System.currentTimeMillis();
                    pathState = 2;
                }
                break;

            case 2: // PICK 2
                if (shootState == ShootState.IDLE) {
                    follower.followPath(pick2);
                    pathState = 3;
                }
                break;

            case 3: // OPEN GATE PATH
                if (!follower.isBusy()) {
                    follower.followPath(opengate);
                    pathState = 4;
                }
                break;

            case 4: // RETURN TO SHOOT FOR BALL 2
                if (!follower.isBusy()) {
                    setShooterVelocity(closeShotVelocity);
                    shootState = ShootState.SPINUP;

                    follower.followPath(shoot2);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    shootState = ShootState.FIRE;
                    fireStartTime = System.currentTimeMillis();
                    pathState = 6;
                }
                break;

            case 6: // PICK 1
                if (shootState == ShootState.IDLE) {
                    follower.followPath(pick1);
                    pathState = 7;
                }
                break;

            case 7: // RETURN TO SHOOT FOR BALL 1
                if (!follower.isBusy()) {
                    setShooterVelocity(closeShotVelocity);
                    shootState = ShootState.SPINUP;

                    follower.followPath(shoot1);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    shootState = ShootState.FIRE;
                    fireStartTime = System.currentTimeMillis();
                    pathState = 9;
                }
                break;

            case 9: // PICK 3
                if (shootState == ShootState.IDLE) {
                    follower.followPath(pick3);
                    pathState = 10;
                }
                break;

            case 10: // RETURN TO SHOOT FOR BALL 3
                if (!follower.isBusy()) {
                    setShooterVelocity(closeShotVelocity);
                    shootState = ShootState.SPINUP;

                    follower.followPath(shoot3);
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    shootState = ShootState.FIRE;
                    fireStartTime = System.currentTimeMillis();
                    pathState = 12;
                }
                break;

            case 12: // PARK
                if (shootState == ShootState.IDLE) {
                    follower.followPath(park);
                    pathState = 13;
                }
                break;

            case 13:
                turret.setTargetPosition(0);
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turret.setPower(0.5);
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("pathState", pathState);
        telemetry.addData("shootState", shootState);
        telemetry.addData("turretPos", turret.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake   = hardwareMap.get(DcMotorEx.class, "intake");
        gate     = hardwareMap.get(Servo.class, "gate");
        turret   = hardwareMap.get(DcMotorEx.class, "turret");

        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        gate.setPosition(GATE_CLOSED);

        // === TURRET INIT ===
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setTargetPosition(TURRET_START_TICKS);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(0.5);

        // === BUILD PATHS AFTER FOLLOWER IS CREATED ===
        buildPaths();
        follower.setStartingPose(startPose);

        pathState = -1; // turret init state
    }

    @Override
    public void start() {}

    @Override
    public void stop() {}
}
