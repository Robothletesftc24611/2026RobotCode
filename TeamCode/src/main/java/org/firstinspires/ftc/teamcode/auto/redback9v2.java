package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Disabled
@Autonomous (name = "red 9 ball back", group = "Iterative Opmode")

public class redback9v2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public Limelight3A limelight;
    public CRServo turretServo;
    public DcMotorEx shooter1, shooter2;
    public DcMotor intake;
    public Servo door, spindexer, scooper;
    public ColorSensor colorSensor;

    private enum ShooterState {
        IDLE,
        SPINUP,
        ALIGN_SLOT,
        FIRE,
        RESET,
        DONE
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private ElapsedTime shooterTimer = new ElapsedTime();

    private int currentShootSlot = 1;
    private double targetVelocity = 1100;
    private double spinup = 2.0;
    private double distance = 0;
    private int currentSlot = 1;

    private enum IntakeState {
        WAIT_FOR_BALL,
        DETECTING,
        LOCKED_IN,
        ADVANCING,
        FULL
    }

    private enum BallColor { PURPLE, GREEN, UNKNOWN }

    private IntakeState intakeState = IntakeState.WAIT_FOR_BALL;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private BallColor detectedColor = BallColor.UNKNOWN;
    private int stableCount = 0;
    private final int REQUIRED_STABLE_FRAMES = 2;

    // Servo positions (use SAME values)
    private double SLOT1_POSs = 0.25;
    private double SLOT2_POSs = 0.45;
    private double SLOT3_POSs = 0.67;

    private final double SLOT1_POS = 0.13;
    private final double SLOT2_POS = 0.33;
    private final double SLOT3_POS = 0.55;
    private final double BLOCKED_NEUTRAL_POS = 0.67;

    private static final double NOMINAL_VOLTAGE = 11.91;
    private static final double kP = 12.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 12.5;

    private BallColor slotOne = BallColor.UNKNOWN;
    private BallColor slotTwo = BallColor.UNKNOWN;
    private BallColor slotThree = BallColor.UNKNOWN;

    private final double LIMELIGHT_OFFSET = -1.0;
    private final double DEADZONE = 1.0;
    private final double MAX_POWER = 1.0;

    private VoltageSensor batteryVoltage;

    private final Pose startPose = new Pose(88.00, 9.00, Math.toRadians(270));
    private final Pose lineup1 = new Pose(100.56,35.42, Math.toRadians(180));
    private final Pose intake1a = new Pose(106.5,35.34, Math.toRadians(180));
    private final Pose intake2a = new Pose(113.65, 35.34, Math.toRadians(180));
    private final Pose intake3a = new Pose(125.98, 35.34, Math.toRadians(180));
    private final Pose lineup = new Pose(101.31, 59.76, Math.toRadians(180));
    private final Pose intake1b = new Pose(108.29, 59.57, Math.toRadians(180));
    private final Pose intake2b = new Pose(113.64, 59.69, Math.toRadians(180));
    private final Pose intake3b = new Pose(125.37, 59.67, Math.toRadians(180));
    private final Pose park = new Pose(101.67, 48.21, Math.toRadians(270));

    private PathChain line1, graba1, graba2, graba3, scorea, line2, grabb1, grabb2, grabb3, scoreb, leave;

    public void buildPaths(){
        line1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, lineup1))
                .addParametricCallback(0.5, () -> intake.setPower(-1.0))
                .addParametricCallback(0.5, () -> door.setPosition(0.7))
                .setLinearHeadingInterpolation(startPose.getHeading(), lineup1.getHeading())
                .build();
        graba1 = follower.pathBuilder()
                .addPath(new BezierLine(lineup1, intake1a))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        graba2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1a, intake2a))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        graba3 = follower.pathBuilder()
                .addPath(new BezierLine(intake2a, intake3a))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorea = follower.pathBuilder()
                .addPath(new BezierLine(intake3a, startPose))
                .addParametricCallback(0.1, () -> intake.setPower(0.0))
                .addParametricCallback(0.1, () -> door.setPosition(1.0))
                .setLinearHeadingInterpolation(intake3a.getHeading(), startPose.getHeading())
                .build();
        line2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, lineup))
                .addParametricCallback(0.6, () -> intake.setPower(-1.0))
                .addParametricCallback(0.6, () -> door.setPosition(0.7))
                .setLinearHeadingInterpolation(startPose.getHeading(), lineup.getHeading())
                .build();
        grabb1 = follower.pathBuilder()
                .addPath(new BezierLine(lineup, intake1b))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        grabb2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1b, intake2b))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        grabb3 = follower.pathBuilder()
                .addPath(new BezierLine(intake2b, intake3b))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scoreb = follower.pathBuilder()
                .addPath(new BezierLine(intake3b, startPose))
                .addParametricCallback(0.1, () -> intake.setPower(0.0))
                .addParametricCallback(0.1, () -> door.setPosition(1.0))
                .setLinearHeadingInterpolation(intake3b.getHeading(), startPose.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(startPose, park))
                .setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading())
                .build();
    }

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                startShooterFSM();
                pathState = 1;
                break;
            case 1:
                if (shooterState == ShooterState.IDLE){
                    limelight.stop();
                    intakeState = IntakeState.WAIT_FOR_BALL;
                    currentSlot = 1;

                    slotOne = BallColor.UNKNOWN;
                    slotTwo = BallColor.UNKNOWN;
                    slotThree = BallColor.UNKNOWN;

                    detectedColor = BallColor.UNKNOWN;
                    stableCount = 0;
                    intakeTimer.reset();

                    spindexer.setPosition(SLOT1_POS);
                    follower.followPath(line1);
                    pathState = 30;
                }
                break;
            case 30:
                if (!follower.isBusy()){
                    follower.followPath(graba1);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    pathTimer.resetTimer();
                    pathState = 4;
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3){
                    follower.followPath(graba2);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    pathTimer.resetTimer();
                    pathState = 6;
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3){
                    follower.followPath(graba3);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.followPath(scorea, true);
                    limelight.start();
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    startShooterFSM();
                    pathState = 9;
                }
                break;
            case 9:
                if (shooterState == ShooterState.IDLE){
                    intakeState = IntakeState.WAIT_FOR_BALL;
                    currentSlot = 1;

                    slotOne = BallColor.UNKNOWN;
                    slotTwo = BallColor.UNKNOWN;
                    slotThree = BallColor.UNKNOWN;

                    detectedColor = BallColor.UNKNOWN;
                    stableCount = 0;
                    intakeTimer.reset();

                    spindexer.setPosition(SLOT1_POS);
                    follower.followPath(line2);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()){
                    follower.followPath(grabb1);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()){
                    pathTimer.resetTimer();
                    pathState = 12;
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3){
                    follower.followPath(grabb2);
                    pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()){
                    pathTimer.resetTimer();
                    pathState = 14;
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3){
                    follower.followPath(grabb3);
                    pathState = 15;
                }
                break;
            case 15:
                if (!follower.isBusy()){
                    follower.followPath(scoreb, true);
                    pathState = 100;
                }
                break;
            case 100:
                if (!follower.isBusy()){
                    pathTimer.resetTimer();
                    pathState = 16;
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() >= 3.0){
                    startShooterFSM();
                    pathState = 17;
                }
                break;
            case 17:
                if (shooterState == ShooterState.IDLE){
                    intakeState = IntakeState.WAIT_FOR_BALL;
                    currentSlot = 1;

                    slotOne = BallColor.UNKNOWN;
                    slotTwo = BallColor.UNKNOWN;
                    slotThree = BallColor.UNKNOWN;

                    detectedColor = BallColor.UNKNOWN;
                    stableCount = 0;
                    intakeTimer.reset();

                    spindexer.setPosition(SLOT1_POS);
                    follower.followPath(leave);
                    pathState = 18;
                }
                break;
            case 18:
                break;
        }
        handleIntakeAuto();
    }
    private void startShooterFSM() {
        shooterState = ShooterState.SPINUP;
        shooterTimer.reset();
        currentShootSlot = 1;
    }

    private void turretControl() {
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

        if (!fiducials.isEmpty()) {
            double tx = llResult.getTx();
            double correctedTx = tx - LIMELIGHT_OFFSET;

            if (Math.abs(correctedTx) > DEADZONE) {
                double power = -correctedTx / 30.0;
                power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
                turretServo.setPower(power);
            } else {
                turretServo.setPower(0.0);
            }
        } else {
            turretServo.setPower(0.0);
        }
    }

    private double getDistanceFromTag(double ta) {
        return (-33.74145 * ta) + 194.923;
    }

    public void shooterFSM() {

        // ---------------- SAFETY ----------------
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null) return;
        double spinupTime;

        // ---------------- DISTANCE → VELOCITY ----------------
        distance = getDistanceFromTag(llResult.getTa());

        if (distance > 170) {
            targetVelocity = 1225;
            spinupTime = 2.5;
        } else {
            targetVelocity = 1100;
            spinupTime = 1.5;
        }

        // ---------------- FSM ----------------
        switch (shooterState) {

            // -----------------------------------
            case IDLE:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                break;

            // -----------------------------------
            case SPINUP:
                shooter1.setVelocity(targetVelocity);
                shooter2.setVelocity(targetVelocity);

                // preload first slot
                spindexer.setPosition(SLOT1_POSs);

                if (shooterTimer.seconds() >= spinupTime) {
                    shooterState = ShooterState.ALIGN_SLOT;
                    shooterTimer.reset();
                }
                break;

            // -----------------------------------
            case ALIGN_SLOT:
                if (currentShootSlot == 1) {
                    spindexer.setPosition(SLOT1_POSs);
                }
                else if (currentShootSlot == 2) {
                    spindexer.setPosition(SLOT2_POSs);
                }
                else if (currentShootSlot == 3) {
                    // keep your pause
                    if (shooterTimer.seconds() >= 0.5) {
                        spindexer.setPosition(SLOT3_POSs);
                    }
                }

                // FIRE only after alignment is guaranteed
                double requiredAlignTime = (currentShootSlot == 3) ? 0.8 : 0.4;

                if (shooterTimer.seconds() >= requiredAlignTime) {
                    shooterState = ShooterState.FIRE;
                    shooterTimer.reset();
                }
                break;


            // -----------------------------------
            case FIRE:
                scooper.setPosition(0.75);   // kick ball

                if (shooterTimer.seconds() >= 0.3) {
                    shooterState = ShooterState.RESET;
                    shooterTimer.reset();
                }
                break;

            // -----------------------------------
            case RESET:
                scooper.setPosition(1.0); // retract

                if (shooterTimer.seconds() > 0.4) {

                    if (currentShootSlot < 3) {
                        currentShootSlot++;          // only increment if more balls remain
                        shooterState = ShooterState.ALIGN_SLOT;
                    } else {
                        shooterState = ShooterState.DONE;
                    }

                    shooterTimer.reset();
                }
                break;



            // -----------------------------------
            case DONE:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);

                if (shooterTimer.seconds() > 1.0){
                    spindexer.setPosition(SLOT1_POS);
                    shooterState = ShooterState.IDLE;
                }
                break;
        }
    }
    private BallColor detectBallColorHSV() {
        float[] hsv = new float[3];
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);

        float h = hsv[0];
        float s = hsv[1];
        float v = hsv[2];

        // Thresholds tuned to your real background values
        if (v < 0.55) return BallColor.UNKNOWN;

        if (h > 190 && h < 260) return BallColor.PURPLE;
        if (h > 120 && h < 190) return BallColor.GREEN;

        return BallColor.UNKNOWN;
    }

    private void handleIntakeAuto() {
        BallColor currentColor = detectBallColorHSV();

        switch (intakeState) {

            case WAIT_FOR_BALL:
                // Optional: ensure intake is actually running while waiting
                // intake.setPower(-1.0);

                if (currentSlot > 3) {
                    intakeState = IntakeState.FULL;
                    break;
                }

                if (currentColor != BallColor.UNKNOWN) {
                    detectedColor = currentColor;
                    stableCount = 1;
                    intakeTimer.reset();              // IMPORTANT: start debounce window now
                    intakeState = IntakeState.DETECTING;
                }
                break;

            case DETECTING:
                // Shorter debounce → more sensitive
                if (intakeTimer.milliseconds() < 150) break;

                if (currentColor == detectedColor && currentColor != BallColor.UNKNOWN) {
                    stableCount++;
                } else {
                    // Lost color or changed → restart
                    intakeState = IntakeState.WAIT_FOR_BALL;
                    break;
                }

                // Make this easier to satisfy
                if (stableCount >= 2) {              // was REQUIRED_STABLE_FRAMES
                    intakeState = IntakeState.LOCKED_IN;
                }
                break;

            case LOCKED_IN:
                if (detectedColor == BallColor.UNKNOWN) {
                    intakeState = IntakeState.WAIT_FOR_BALL;
                    break;
                }

                // Record ball into current slot
                if (currentSlot == 1)      slotOne  = detectedColor;
                else if (currentSlot == 2) slotTwo  = detectedColor;
                else if (currentSlot == 3) slotThree = detectedColor;

                intakeTimer.reset();
                intakeState = IntakeState.ADVANCING;
                break;

            case ADVANCING:
                // Shorter advance delay so we don't miss the next ball
                if (intakeTimer.milliseconds() > 250) {
                    if (currentSlot == 1)      spindexer.setPosition(SLOT2_POS);
                    else if (currentSlot == 2) spindexer.setPosition(SLOT3_POS);
                    else if (currentSlot == 3) spindexer.setPosition(BLOCKED_NEUTRAL_POS);

                    currentSlot++;
                    intakeState = (currentSlot > 3) ? IntakeState.FULL : IntakeState.WAIT_FOR_BALL;
                    intakeTimer.reset();
                }
                break;

            case FULL:
                // Optional: stop intake when full
                // intake.setPower(0.0);
                break;

            // ===== DEBUG TELEMETRY FOR INTAKE + SPINDEXER ====

        }
    }


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turret");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        door = hardwareMap.get(Servo.class, "door");

        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        spindexer = hardwareMap.get(Servo.class, "spindexer");
        scooper = hardwareMap.get(Servo.class, "kicker");
        spindexer.setPosition(SLOT1_POSs);

        limelight.pipelineSwitch(1);
        limelight.start();

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        double compensatedF = kF * (NOMINAL_VOLTAGE / batteryVoltage.getVoltage());

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter1.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, compensatedF)
        );

        shooter2.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, compensatedF)
        );

        scooper.setPosition(1.0);
        door.setPosition(1.0);

        pathState = 0;


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    @Override
    public void loop(){

        double voltage = batteryVoltage.getVoltage();
        double dynamicF = kF * (NOMINAL_VOLTAGE / voltage);

        shooter1.setVelocityPIDFCoefficients(kP, kI, kD, dynamicF);
        shooter2.setVelocityPIDFCoefficients(kP, kI, kD, dynamicF);

        follower.update();
        autonomousPathUpdate();
        turretControl();
        shooterFSM();

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addLine("=== GLOBAL SPINDEXER ===");
        telemetry.addData("Spindexer Pos", spindexer.getPosition());
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Current Slot", currentSlot);
// ===== DEBUG TELEMETRY FOR INTAKE + SPINDEXER =====
        telemetry.addLine("=== INTAKE DEBUG ===");
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Current Slot", currentSlot);

        telemetry.addData("Locked Color", detectedColor);
        telemetry.addData("Stable Count", stableCount);
        telemetry.addData("Intake Timer (ms)", intakeTimer.milliseconds());

        telemetry.addLine("Slots:");
        telemetry.addData("Slot 1", slotOne);
        telemetry.addData("Slot 2", slotTwo);
        telemetry.addData("Slot 3", slotThree);

        telemetry.addLine("Spindexer:");
        telemetry.addData("Spindexer Pos", spindexer.getPosition());
        telemetry.addData("Target Pos",
                (currentSlot == 1 ? SLOT1_POS :
                        currentSlot == 2 ? SLOT2_POS :
                                currentSlot == 3 ? SLOT3_POS :
                                        BLOCKED_NEUTRAL_POS));

        telemetry.update();

    }
}
