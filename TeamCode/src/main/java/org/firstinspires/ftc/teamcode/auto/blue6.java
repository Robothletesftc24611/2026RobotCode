package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Disabled
@Autonomous(name = "BLUE FRONT SORTED - 6")
public class blue6 extends OpMode {

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
    private final Pose startPose = new Pose(-1, -2, Math.toRadians(270));

    private final Pose readmotif = new Pose (24, -24, Math.toRadians(225));
    private final Pose lineup1 = new Pose (24, -58, Math.toRadians(0));
    private final Pose pick1a = new Pose (14, -58, Math.toRadians(0));
    private final Pose pick1b = new Pose(6, -58, Math.toRadians(0));
    private final Pose pick1c = new Pose(-6, -58, Math.toRadians(0));
    private final Pose last = new Pose(-4, -60, Math.toRadians(0));

    private final Pose shootpose = new Pose(30, -53, Math.toRadians(305));
    private Path read;

    private PathChain shoot1, line1, pickup1a, pickup1b, pickup1c, shoot2, park;

    // Shooting / spindexer
    private final double[] spindexerPositions = {-0.1, 0.4, 0.85};
    private int shootState = 3;
    private int ballsShot = 0;
    private boolean shootingDone = true;

    // shooter PIDF constants (copied from teleop)
    private static final double NOMINAL_VOLTAGE = 11.38;
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 22.0; // base feedforward constant

    // turret / limelight tuning
    private final double LIMELIGHT_OFFSET = -1.0;
    private final double DEADZONE = 1.0;
    private final double MAX_POWER = 1.0;

    private ColorType lastColor = ColorType.UNKNOWN;
    private int stableCount = 0;
    private final int REQUIRED_STABLE_FRAMES = 3;

    private static final float PURPLE_HUE_MIN = 190;
    private static final float PURPLE_HUE_MAX = 250;

    private static final float GREEN_HUE_MIN = 130;
    private static final float GREEN_HUE_MAX = 165;

    private static final float MIN_SAT = 0.4f;
    private static final float MIN_VAL = 0.2f;


    // timers
    private ElapsedTime actionTimer = new ElapsedTime();
    private ElapsedTime opmodeTimer = new ElapsedTime();
    enum ColorType { GREEN, PURPLE, UNKNOWN }

    // Motif shooter FSM
    private enum MotifState {
        IDLE,
        SPINUP,
        PRESENT,
        READ,
        MATCH_CHECK,
        SHOOT,
        RETRACT,
        ADVANCE,
        DONE
    }

    private MotifState motifState = MotifState.IDLE;
    private int motifIndex = 0;
    private int pocketIndex = 0;

    private ElapsedTime motifTimer = new ElapsedTime();
    private boolean motifShootDone = false;


    private ColorType[] motifPattern = null;
    private boolean motifDetected = false;

    private ColorSensor sensor;



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
        sensor = hardwareMap.get(ColorSensor.class, "color");

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
        limelight.pipelineSwitch(0);
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

    public static class HSV {
        public float h, s, v;
        public HSV(float h, float s, float v) {
            this.h = h;
            this.s = s;
            this.v = v;
        }
    }

    public HSV rgbToHSV(int r, int g, int b) {
        float rf = r / 255f;
        float gf = g / 255f;
        float bf = b / 255f;

        float max = Math.max(rf, Math.max(gf, bf));
        float min = Math.min(rf, Math.min(gf, bf));
        float delta = max - min;

        float h;
        if (delta == 0) {
            h = 0;
        } else if (max == rf) {
            h = 60 * (((gf - bf) / delta) % 6);
        } else if (max == gf) {
            h = 60 * (((bf - rf) / delta) + 2);
        } else {
            h = 60 * (((rf - gf) / delta) + 4);
        }
        if (h < 0) h += 360;

        float s = (max == 0) ? 0 : (delta / max);
        float v = max;

        return new HSV(h, s, v);
    }


    private ColorType readBallColor() {

        // Read raw RGB from your color sensor
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        // Convert to HSV
        HSV hsv = rgbToHSV(r, g, b);

        // Basic sanity checks
        if (hsv.s < MIN_SAT || hsv.v < MIN_VAL) {
            return ColorType.UNKNOWN;
        }

        float h = hsv.h;

        // Classify based on tuned hue ranges
        ColorType raw;
        if (h >= PURPLE_HUE_MIN && h <= PURPLE_HUE_MAX) {
            raw = ColorType.PURPLE;
        } else if (h >= GREEN_HUE_MIN && h <= GREEN_HUE_MAX) {
            raw = ColorType.GREEN;
        } else {
            raw = ColorType.UNKNOWN;
        }

        // Stable 3‑frame filter
        if (raw == lastColor) {
            stableCount++;
        } else {
            stableCount = 0;
            lastColor = raw;
        }

        if (stableCount >= REQUIRED_STABLE_FRAMES) {
            return raw;
        }

        return ColorType.UNKNOWN;
    }


    private void detectMotifFromTag() {

        List<LLResultTypes.FiducialResult> tags =
                limelight.getLatestResult().getFiducialResults();

        if (tags == null || tags.isEmpty()) {
            return; // nothing seen yet
        }

        int id = tags.get(0).getFiducialId();

        switch (id) {

            case 21: // GPP
                motifPattern = new ColorType[]{
                        ColorType.GREEN,
                        ColorType.PURPLE,
                        ColorType.PURPLE
                };
                motifDetected = true;
                break;

            case 22: // PGP
                motifPattern = new ColorType[]{
                        ColorType.PURPLE,
                        ColorType.GREEN,
                        ColorType.PURPLE
                };
                motifDetected = true;
                break;

            case 23: // PPG
                motifPattern = new ColorType[]{
                        ColorType.PURPLE,
                        ColorType.PURPLE,
                        ColorType.GREEN
                };
                motifDetected = true;
                break;

            default:
                // ignore other tags
                break;
        }
    }


    private void buildPaths() {
        read = new Path(new BezierLine(startPose, readmotif));
        read.setLinearHeadingInterpolation(startPose.getHeading(), readmotif.getHeading());

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(readmotif, shootpose))
                .setLinearHeadingInterpolation(readmotif.getHeading(), shootpose.getHeading())
                .build();
        line1 = follower.pathBuilder()
                .addPath(new BezierLine(shootpose, lineup1))
                .setLinearHeadingInterpolation(shootpose.getHeading(), lineup1.getHeading())
                .addParametricCallback(0.2, () -> door.setPosition(0.0))
                .addParametricCallback(0.2, () -> intake.setPower(-0.6))
                .addParametricCallback(0.2, () -> spindexer.setPosition(0.1))
                .build();
        pickup1a = follower.pathBuilder()
                .addPath(new BezierLine(lineup1, pick1a))
                .setLinearHeadingInterpolation(lineup1.getHeading(), pick1a.getHeading())
                .addParametricCallback(0.9, () -> spindexer.setPosition(0.5))
                .build();
        pickup1b = follower.pathBuilder()
                .addPath(new BezierLine(pick1a, pick1b))
                .setLinearHeadingInterpolation(pick1a.getHeading(), pick1b.getHeading())
                .addParametricCallback(0.9, () -> spindexer.setPosition(1.0))
                .build();
        pickup1c = follower.pathBuilder()
                .addPath(new BezierLine(pick1b, pick1c))
                .setLinearHeadingInterpolation(pick1b.getHeading(), pick1c.getHeading())
                .addParametricCallback(0.9, () -> spindexer.setPosition(0.1))
                .addParametricCallback(0.9, () -> door.setPosition(0.2))
                .addParametricCallback(0.9, () -> intake.setPower(0.0))
                .build();
        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(pick1c, shootpose))
                .setLinearHeadingInterpolation(pick1c.getHeading(), shootpose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootpose, last))
                .setLinearHeadingInterpolation(shootpose.getHeading(), last.getHeading())
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
        motifShooterUpdate(); // uses shooter.setVelocity(...) and spindexer/scooper timing

        telemetry.addLine("=== MOTIF DEBUG ===");

        telemetry.addData("motifDetected", motifDetected);
        telemetry.addData("motifPattern",
                motifPattern == null ? "NONE" :
                        motifPattern[0] + " " + motifPattern[1] + " " + motifPattern[2]);

        telemetry.addData("motifState", motifState);
        telemetry.addData("motifIndex", motifIndex);
        telemetry.addData("motifShootDone", motifShootDone);

        telemetry.addLine();

        telemetry.addLine("=== COLOR SENSOR ===");
        telemetry.addData("raw R", sensor.red());
        telemetry.addData("raw G", sensor.green());
        telemetry.addData("raw B", sensor.blue());

        HSV hsv = rgbToHSV(sensor.red(), sensor.green(), sensor.blue());
        telemetry.addData("HSV H", hsv.h);
        telemetry.addData("HSV S", hsv.s);
        telemetry.addData("HSV V", hsv.v);

        telemetry.addData("classifiedColor", readBallColor());
        telemetry.addData("stableCount", stableCount);

        telemetry.addLine();

        telemetry.addLine("=== SPINDEXER ===");
        telemetry.addData("spindexerPos", spindexer.getPosition());
        telemetry.addData("targetPocket", motifIndex < 3 ? spindexerPositions[motifIndex] : -1);

        telemetry.addLine();

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("shooterTargetVel", getExpectedShooterVelocityForRange());
        telemetry.addData("shooterActualVel", shooter.getVelocity());

        telemetry.addLine();

        telemetry.addLine("=== PATH ===");
        telemetry.addData("pathState", pathState);
        telemetry.addData("followerBusy", follower.isBusy());
        telemetry.addData("pose",
                follower.getPose().getX() + ", " +
                        follower.getPose().getY() + ", " +
                        Math.toDegrees(follower.getPose().getHeading()));


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
                // Drive to the AprilTag reading position
                follower.followPath(read);
                pathState = 1;
                break;

            case 1:
                // Wait until robot reaches the tag-reading pose
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                // Actively try to detect the motif
                detectMotifFromTag();

                telemetry.addData("Detecting Tag...", "");
                telemetry.addData("motifDetected", motifDetected);
                telemetry.addData("motifPattern",
                        motifPattern == null ? "NONE" :
                                motifPattern[0] + " " + motifPattern[1] + " " + motifPattern[2]);
                telemetry.update();

                if (motifDetected) {
                    pathState = 3;
                }
                break;

            case 3:
                telemetry.addLine("MOTIF LOCKED IN!");
                telemetry.addData("Pattern",
                        motifPattern[0] + " " + motifPattern[1] + " " + motifPattern[2]);
                telemetry.update();
                limelight.pipelineSwitch(2);

                // Continue with the rest of autonomous
                pathState = 4;
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.followPath(shoot1);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    motifState = MotifState.IDLE;
                    motifShootDone = false;
                    pathState = 6;
                }
                break;
            case 6:
                if (motifShootDone){
                    follower.followPath(line1);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.4);
                    follower.followPath(pickup1a);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    opmodeTimer.reset();
                    pathState = 9;
                }
                break;
            case 9:
                if (opmodeTimer.seconds() > 0.5){
                    follower.followPath(pickup1b);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()){
                    opmodeTimer.reset();
                    pathState = 11;
                }
                break;
            case 11:
                if (opmodeTimer.seconds() > 0.5){
                    follower.followPath(pickup1c);
                    pathState = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()){
                    follower.followPath(shoot2);
                    pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()){
                    motifState = MotifState.IDLE;
                    motifShootDone = false;
                    pathState = 14;
                }
                break;
            case 14:
                if (motifShootDone){
                    follower.followPath(park);
                    pathState = 15;
                }
                break;
            case 15:
                break;

        }
    }


    // Helper to pick target velocity based on limelight area (Ta)
    private double getExpectedShooterVelocityForRange() {
        // fallback default
        double defaultVelocity = 1025;
        try {
            LLResult llResult = limelight.getLatestResult();
            double ta = llResult.getTa(); // target area
            // adopt simple linear mapping similar to TeleOp example:
            double distance = (-33.74145 * ta) + 194.923;
            if (distance > 170) return 1025;
            else return 1025;
        } catch (Exception e) {
            return defaultVelocity;
        }
    }

    // Shooting FSM that uses velocity control and the spindexer/scooper servos

    private void motifShooterUpdate() {

        if (motifShootDone || motifPattern == null) return;

        switch (motifState) {

            case IDLE:
                motifIndex = 0;
                pocketIndex = 0;
                applyShooterPIDFCompensated();
                shooter.setVelocity(getExpectedShooterVelocityForRange());
                motifTimer.reset();
                motifState = MotifState.SPINUP;
                break;

            case SPINUP:
                if (motifTimer.seconds() > 1.0) {
                    motifState = MotifState.PRESENT;
                }
                break;

            case PRESENT:
                switch (pocketIndex) {
                    case 0: spindexer.setPosition(spindexerPositions[0]); break;
                    case 1: spindexer.setPosition(spindexerPositions[1]); break;
                    case 2: spindexer.setPosition(spindexerPositions[2]); break;
                }
                motifTimer.reset();
                motifState = MotifState.READ;
                break;

            case READ:
                if (motifTimer.seconds() > 0.75) {
                    motifState = MotifState.MATCH_CHECK;
                }
                break;

            case MATCH_CHECK:
                ColorType seen = readBallColor();
                ColorType target = motifPattern[motifIndex];

                // IMPORTANT: UNKNOWN guard
                if (seen == ColorType.UNKNOWN) {
                    pocketIndex = (pocketIndex + 1) % 3;
                    motifState = MotifState.PRESENT;
                    break;// stay in MATCH_CHECK until stable
                }

                if (seen == target) {
                    scooper.setPosition(0.5);
                    motifTimer.reset();
                    motifState = MotifState.SHOOT;
                } else {
                    pocketIndex = (pocketIndex + 1) % 3;
                    motifState = MotifState.PRESENT;
                }
                break;

            case SHOOT:
                if (motifTimer.seconds() > 0.25) {
                    scooper.setPosition(0.0);
                    motifTimer.reset();
                    motifState = MotifState.RETRACT;
                }
                break;

            case RETRACT:
                if (motifTimer.seconds() > 0.5) {
                    motifIndex++;

                    if (motifIndex >= 3) {
                        shooter.setVelocity(0);
                        spindexer.setPosition(spindexerPositions[0]);
                        motifShootDone = true;
                        motifState = MotifState.DONE;
                    } else {
                        pocketIndex = 0;
                        motifState = MotifState.PRESENT;
                    }
                }
                break;

            case DONE:
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
