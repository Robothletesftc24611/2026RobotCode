package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue back gate cycles", group = "COMPETITION")
public class bluebackgatecycles extends OpMode {

    // === PATHING ===
    private Follower follower;
    private int pathState;

    // === SHOOTER SYSTEM ===
    private DcMotorEx shooter1, shooter2;
    private DcMotorEx intake;
    private Servo gate;

    private final double shooter_kP = 20;
    private final double shooter_kF = 12;
    private final double nominalVoltage = 13.2;
    private final double closeShotVelocity = 1750;

    private static final double GATE_CLOSED = 0.6;
    private static final double GATE_OPEN = 0.85;

    // === TURRET ===
    private DcMotorEx turret;
    private static final int TURRET_START_TICKS = 50; //TUNE // example: 180 degrees if 4.5 ticks/deg

    // === SHOOT FSM ===
    private enum ShootState { IDLE, SPINUP, FIRE, DONE }
    private ShootState shootState = ShootState.IDLE;
    private long fireStartTime = 0;

    // === POSES ===
    private final Pose shootPose   = new Pose(57.25, 8.5, Math.toRadians(270));
    private final Pose pickhuman    = new Pose(14, 8.6, Math.toRadians(0));
    private final Pose backhuman    = new Pose(28, 9.1, Math.toRadians(0));
    private final Pose repick    = new Pose(14, 9.1, Math.toRadians(0));
    private final Pose gatecycle1    = new Pose(14, 15, Math.toRadians(0));
    private final Pose gatecycle1back    = new Pose(23, 14, Math.toRadians(0));
    private final Pose gatecyclerepick  = new Pose(14, 14, Math.toRadians(0));
    private final Pose finalPose   = new Pose(34, 11, Math.toRadians(0));

    private PathChain pickuphuman, shoothuman, pickupgate, shootgate, park;

    // === BUILD PATHS (your original structure preserved) ===
    public void buildPaths() {
        pickuphuman = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickhuman))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickhuman.getHeading())
                .addParametricCallback(0.1, () -> intake.setPower(-1.0))
                .addPath(new BezierLine(pickhuman, backhuman))
                .setLinearHeadingInterpolation(pickhuman.getHeading(), backhuman.getHeading())
                .addPath(new BezierLine(backhuman, repick))
                .setLinearHeadingInterpolation(backhuman.getHeading(), repick.getHeading())
                .build();
        shoothuman = follower.pathBuilder()
                .addPath(new BezierLine(repick, shootPose))
                .setLinearHeadingInterpolation(repick.getHeading(), shootPose.getHeading())
                .build();
        pickupgate = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gatecycle1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gatecycle1.getHeading())
                .addParametricCallback(0.1, () -> intake.setPower(-1.0))
                .addPath(new BezierLine(gatecycle1, gatecycle1back))
                .setLinearHeadingInterpolation(gatecycle1.getHeading(), gatecycle1back.getHeading())
                .addPath(new BezierLine(gatecycle1back, gatecyclerepick))
                .setLinearHeadingInterpolation(gatecycle1back.getHeading(), gatecyclerepick.getHeading())
                .build();
        shootgate = follower.pathBuilder()
                .addPath(new BezierLine(gatecyclerepick, shootPose))
                .setLinearHeadingInterpolation(gatecyclerepick.getHeading(), shootPose.getHeading())
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

                if (elapsed > 250) intake.setPower(-0.8);
                else intake.setPower(0);

                if (elapsed > 2000) shootState = ShootState.DONE;
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

            // === 0: INITIAL SHOOT PRELOAD ===
            case 0:
                follower.setMaxPower(0.7);

                setShooterVelocity(closeShotVelocity);
                gate.setPosition(GATE_CLOSED);
                shootState = ShootState.SPINUP;

                if (Math.abs(shooter2.getVelocity() - closeShotVelocity) < 50) {
                    shootState = ShootState.FIRE;
                    fireStartTime = System.currentTimeMillis();
                    pathState = 1;
                }
                break;

            // === 1: DRIVE TO HUMAN PLAYER PICKUP ===
            case 1:
                if (shootState == ShootState.IDLE) {
                    follower.followPath(pickuphuman, true);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy() && shootState == ShootState.IDLE) {
                    intake.setPower(0);
                    follower.followPath(shoothuman, true);
                    pathState = 3;
                }
                break;

            // === 3: SHOOT HUMAN PLAYER BALL ===
            case 3:
                if (!follower.isBusy()) {

                    // If we haven't started spinup yet, start it
                    if (shootState == ShootState.IDLE) {
                        setShooterVelocity(closeShotVelocity);
                        shootState = ShootState.SPINUP;
                    }

                    // Now wait until shooter actually reaches velocity
                    if (shootState == ShootState.SPINUP &&
                            Math.abs(shooter2.getVelocity() - closeShotVelocity) < 50) {

                        shootState = ShootState.FIRE;
                        fireStartTime = System.currentTimeMillis();
                        pathState = 4;
                    }
                }
                break;


            // === 4: START GATE CYCLE LOOP ===
            case 4:
                if (shootState == ShootState.IDLE) {
                    follower.followPath(pickupgate, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy() && shootState == ShootState.IDLE) {
                    follower.followPath(shootgate, true);
                    pathState = 6;
                }
                break;

            // === 6: SHOOT GATE BALL ===
            case 6:
                if (!follower.isBusy()) {

                    // If we haven't started spinup yet, start it
                    if (shootState == ShootState.IDLE) {
                        setShooterVelocity(closeShotVelocity);
                        shootState = ShootState.SPINUP;
                    }

                    // Now wait until shooter actually reaches velocity
                    if (shootState == ShootState.SPINUP &&
                            Math.abs(shooter2.getVelocity() - closeShotVelocity) < 50) {

                        shootState = ShootState.FIRE;
                        fireStartTime = System.currentTimeMillis();
                        pathState = 7;
                    }
                }
                break;

            // === 7: LOOP GATE CYCLES UNTIL 10 SEC LEFT ===
            case 7:
                double timeElapsed = getRuntime();

                if (timeElapsed < 20.0) {
                    if (shootState == ShootState.IDLE) {
                        follower.followPath(pickupgate, true);
                        pathState = 8;
                    }
                } else {
                    if (shootState == ShootState.IDLE) {
                        follower.followPath(park, true);
                        pathState = 99;
                    }
                }
                break;

            case 8:
                if (!follower.isBusy() && shootState == ShootState.IDLE) {
                    follower.followPath(shootgate, true);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy() && shootState == ShootState.IDLE) {
                    setShooterVelocity(closeShotVelocity);
                    shootState = ShootState.SPINUP;

                    if (Math.abs(shooter2.getVelocity() - closeShotVelocity) < 50) {
                        shootState = ShootState.FIRE;
                        fireStartTime = System.currentTimeMillis();
                        pathState = 7; // loop back
                    }
                }
                break;

            // === 99: PARK + RESET TURRET ===
            case 99:
                if (!follower.isBusy() && shootState == ShootState.IDLE) {
                    turret.setTargetPosition(0);
                    turret.setPower(0.5);
                    pathState = 100;
                }
                break;

            case 100:
                // End state
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
        follower.setStartingPose(shootPose);

        pathState = 0; // turret init state
    }

    @Override
    public void start() {}

    @Override
    public void stop() {}
}
