package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
@Disabled
@TeleOp(name="FINAL BLUE TELE_OP", group="Iterative OpMode")
public class bluefinal extends OpMode {

    // ============================================================
    //  HARDWARE
    // ============================================================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor intake;
    private DcMotorEx shooter;
    private Servo spindexer, scooper, door;
    private CRServo turretServo;
    private Limelight3A limelight;
    private ColorSensor colorSensor;
    private DigitalChannel ballSensor;
    private VoltageSensor batteryVoltage;

    // ============================================================
    //  SHOOTER PIDF CONSTANTS
    // ============================================================
    private static final double NOMINAL_VOLTAGE = 11.38;
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 22.5;

    private double targetVelocity = 0;
    private double distance = 0;

    // ============================================================
    //  LIMELIGHT CONSTANTS
    // ============================================================
    private final double LIMELIGHT_OFFSET = 1.0;
    private final double DEADZONE = 1.0;
    private final double MAX_POWER = 1.0;

    // ============================================================
    //  NORMAL SHOOTER FSM
    // ============================================================
    private enum ShooterState { IDLE, SPINUP, INDEX_BALL, SCOOP, RESET, DONE }
    private ShooterState shooterState = ShooterState.IDLE;
    private ElapsedTime shooterTimer = new ElapsedTime();
    private int ballCount = 0;

    // ============================================================
    //  SPINDEXER FSM
    // ============================================================
    private enum SpindexerState { IDLE, BALL_DETECTED, WAIT_DELAY, ADVANCE, DONE }
    private SpindexerState spindexerState = SpindexerState.IDLE;
    private ElapsedTime spindexerTimer = new ElapsedTime();
    private int currentPositionIndex = 0;
    private boolean buttonPreviouslyPressed = false;
    double[] positions = {0.1, 0.5, 1};

    // ============================================================
    //  MOTIF ENUMS (OUTER SO JAVA ALLOWS THEM)
    // ============================================================
    private enum MotifBallColor { PURPLE, GREEN, UNKNOWN }
    private enum MotifState { SELECTING, WAIT_FOR_Y, SPIN_UP, SEEKING, ALIGNING, FIRING, ADVANCE, DONE }

    // ============================================================
    //  MOTIF MODE TOGGLE
    // ============================================================
    private boolean motifMode = false;
    private boolean motifTogglePressed = false;

    private MotifFSM motifFSM = new MotifFSM();

    // ============================================================
    //  INIT
    // ============================================================
    @Override
    public void init() {

        // Drive
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Subsystems
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        spindexer = hardwareMap.get(Servo.class, "spindexer");
        scooper = hardwareMap.get(Servo.class, "scooper");
        door = hardwareMap.get(Servo.class, "door");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        ballSensor = hardwareMap.get(DigitalChannel.class, "ballSensor");
        ballSensor.setMode(DigitalChannel.Mode.INPUT);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        // Voltage compensation
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        double compensatedF = kF * (NOMINAL_VOLTAGE / batteryVoltage.getVoltage());
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, compensatedF));

        // Servo defaults
        scooper.setDirection(Servo.Direction.REVERSE);
        spindexer.setPosition(0.1);
        scooper.setPosition(0.0);
        door.setPosition(0.0);

        telemetry.addData("Status", "Initialized");
    }

    // ============================================================
    //  MAIN LOOP
    // ============================================================
    @Override
    public void loop() {

        // ---------------------------
        // MOTIF MODE TOGGLE
        // ---------------------------
        if (gamepad2.y && !motifTogglePressed) {
            motifMode = !motifMode;
            motifTogglePressed = true;
        }
        if (!gamepad2.y) motifTogglePressed = false;

        // ---------------------------
        // NORMAL TELEOP (motif OFF)
        // ---------------------------
        if (!motifMode) {

            telemetry.addData("Shooter Mode", "NORMAL");

            driveControl();
            intakeControl();
            turretControl();
            manualTurretControl();
            shooterFSM();
            spindexerFSM();
            spindexerManualControl();
            telemetryOutput();
            return;
        }

        // ---------------------------
        // MOTIF MODE ACTIVE
        // ---------------------------
        telemetry.addData("Shooter Mode", "MOTIF");

        driveControl();
        intakeControl();
        turretControl();
        manualTurretControl();

        motifFSM.update();

        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.addData("Distance (LL)", distance);

        telemetry.update();
    }

    // ============================================================
    //  DRIVE CONTROL
    // ============================================================
    private void driveControl() {
        double axial = gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        double speedLimit = (gamepad1.right_trigger > 0) ? 0.9 : 0.6;
        fl = Range.clip(fl, -speedLimit, speedLimit);
        fr = Range.clip(fr, -speedLimit, speedLimit);
        bl = Range.clip(bl, -speedLimit, speedLimit);
        br = Range.clip(br, -speedLimit, speedLimit);

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    // ============================================================
    //  INTAKE CONTROL
    // ============================================================
    private void intakeControl() {
        if (gamepad2.right_bumper) {
            intake.setPower(-0.5);
            door.setPosition(-0.5);
        }
        else if (gamepad2.a){
            intake.setPower(0.5);
            door.setPosition(-0.5);
        } else {
            intake.setPower(0);
            door.setPosition(0.2);
        }
    }

    // ============================================================
    //  TURRET CONTROL
    // ============================================================
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

    private void manualTurretControl(){
        if (gamepad2.dpad_left) turretServo.setPower(1.0);
        else if (gamepad2.dpad_right) turretServo.setPower(-1.0);
    }

    // ============================================================
    //  NORMAL SHOOTER FSM
    // ============================================================
    private void shooterFSM() {
        LLResult llResult = limelight.getLatestResult();
        distance = getDistanceFromTag(llResult.getTa());

        if (distance > 170){
            targetVelocity = 1400;
        } else {
            targetVelocity = 1150;
        }

        if (gamepad2.x && shooterState == ShooterState.IDLE) {
            shooterState = ShooterState.SPINUP;
            shooterTimer.reset();
            ballCount = 0;
        }

        switch (shooterState) {
            case SPINUP:
                shooter.setVelocity(targetVelocity);
                if (shooterTimer.seconds() > 2.0) {
                    shooterState = ShooterState.INDEX_BALL;
                    shooterTimer.reset();
                }
                break;

            case INDEX_BALL:
                spindexer.setPosition(ballCount == 0 ? -0.1 :
                        ballCount == 1 ? 0.4 : 0.85);
                if (shooterTimer.seconds() > 1.0) {
                    shooterState = ShooterState.SCOOP;
                    shooterTimer.reset();
                }
                break;

            case SCOOP:
                scooper.setPosition(0.5);
                if (shooterTimer.seconds() > 0.5) {
                    shooterState = ShooterState.RESET;
                    shooterTimer.reset();
                }
                break;

            case RESET:
                scooper.setPosition(0.0);
                ballCount++;
                shooterState = (ballCount >= 3) ? ShooterState.DONE : ShooterState.INDEX_BALL;
                shooterTimer.reset();
                break;

            case DONE:
                shooter.setVelocity(0);
                spindexer.setPosition(0.1);
                shooterState = ShooterState.IDLE;
                break;

            case IDLE:
            default:
                break;
        }
    }

    private double getDistanceFromTag(double ta) {
        return (-33.74145 * ta) + 194.923;
    }

    // ============================================================
    //  SPINDEXER FSM
    // ============================================================
    private void spindexerFSM() {
        boolean ballDetected = !ballSensor.getState();

        switch (spindexerState) {
            case IDLE:
                if (ballDetected) spindexerState = SpindexerState.BALL_DETECTED;
                break;

            case BALL_DETECTED:
                spindexerTimer.reset();
                spindexerState = SpindexerState.ADVANCE;
                break;

            case WAIT_DELAY:
                if (spindexerTimer.seconds() >= 0.0) spindexerState = SpindexerState.ADVANCE;
                break;

            case ADVANCE:
                currentPositionIndex++;
                if (currentPositionIndex >= positions.length) currentPositionIndex = 0;
                spindexer.setPosition(positions[currentPositionIndex]);
                spindexerState = SpindexerState.DONE;
                break;

            case DONE:
                if (!ballDetected) spindexerState = SpindexerState.IDLE;
                break;
        }
    }

    private void spindexerManualControl(){
        boolean buttonPressed = gamepad2.left_bumper;
        if (buttonPressed && !buttonPreviouslyPressed){
            currentPositionIndex++;
            if (currentPositionIndex >= positions.length) currentPositionIndex = 0;
            spindexer.setPosition(positions[currentPositionIndex]);
        }
        buttonPreviouslyPressed = buttonPressed;
    }

    // ============================================================
    //  TELEMETRY
    // ============================================================
    private void telemetryOutput() {
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Spindexer Position", spindexer.getPosition());
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }

    // ============================================================
    //  MOTIF FSM (AT BOTTOM OF FILE)
    // ============================================================
    private class MotifFSM {

        // ---------------------------
        // HSV CLASSIFIER
        // ---------------------------
        public class HSV {
            public float h, s, v;
            public HSV(float h, float s, float v) {
                this.h = h; this.s = s; this.v = v;
            }
        }

        public HSV rgbToHSV(int r, int g, int b) {
            float rf = r / 255f, gf = g / 255f, bf = b / 255f;
            float max = Math.max(rf, Math.max(gf, bf));
            float min = Math.min(rf, Math.min(gf, bf));
            float delta = max - min;

            float h;
            if (delta == 0) h = 0;
            else if (max == rf) h = 60 * (((gf - bf) / delta) % 6);
            else if (max == gf) h = 60 * (((bf - rf) / delta) + 2);
            else h = 60 * (((rf - gf) / delta) + 4);

            if (h < 0) h += 360;
            float s = (max == 0) ? 0 : (delta / max);
            float v = max;

            return new HSV(h, s, v);
        }

        private static final float PURPLE_HUE_MIN = 170;
        private static final float PURPLE_HUE_MAX = 300;
        private static final float GREEN_HUE_MIN  = 70;
        private static final float GREEN_HUE_MAX  = 165;
        private static final float MIN_SAT = 0.25f;
        private static final float MIN_VAL = 0.15f;

        private int stableMatchCount = 0;
        private final int REQUIRED_STABLE_MATCH = 4;

        public MotifBallColor classify(int r, int g, int b) {
            HSV hsv = rgbToHSV(r, g, b);
            if (hsv.s < MIN_SAT || hsv.v < MIN_VAL) return MotifBallColor.UNKNOWN;

            if (hsv.h >= PURPLE_HUE_MIN && hsv.h <= PURPLE_HUE_MAX) return MotifBallColor.PURPLE;
            if (hsv.h >= GREEN_HUE_MIN && hsv.h <= GREEN_HUE_MAX) return MotifBallColor.GREEN;

            return MotifBallColor.UNKNOWN;
        }

        // ---------------------------
        // FSM VARIABLES
        // ---------------------------
        private MotifState state = MotifState.WAIT_FOR_Y;
        private long stateStart = 0;

        private int motifChoice = 0;
        private MotifBallColor[] motif;

        private int motifIndex = 0;
        private boolean firedOnce = false;

        private int seekIndex = 0;
        private long lastSeekSwitch = 0;
        private final long SEEK_DWELL_MS = 1500;

        private final double POS1 = 0.0;
        private final double POS2 = 0.45;
        private final double POS3 = 0.90;

        private final double SCOOP_EXTEND = 0.5;
        private final double SCOOP_RETRACT = 0.0;

        private void resetTimer() { stateStart = System.currentTimeMillis(); }
        private boolean timePassed(long ms) { return System.currentTimeMillis() - stateStart >= ms; }

        private void loadMotif() {
            switch (motifChoice) {
                case 0: motif = new MotifBallColor[]{ MotifBallColor.PURPLE, MotifBallColor.GREEN, MotifBallColor.PURPLE }; break;
                case 1: motif = new MotifBallColor[]{ MotifBallColor.PURPLE, MotifBallColor.PURPLE, MotifBallColor.GREEN }; break;
                case 2: motif = new MotifBallColor[]{ MotifBallColor.GREEN, MotifBallColor.PURPLE, MotifBallColor.PURPLE }; break;
            }
        }

        private String motifName() {
            switch (motifChoice) {
                case 0: return "PGP";
                case 1: return "PPG";
                case 2: return "GPP";
            }
            return "???";
        }

        // Constructor: default motif
        public MotifFSM() {
            loadMotif();
        }

        public void update() {

            // Compute shooter velocity like normal shooter
            LLResult llResult = limelight.getLatestResult();
            distance = getDistanceFromTag(llResult.getTa());
            if (distance > 170) {
                targetVelocity = 1400;
            } else {
                targetVelocity = 1150;
            }

            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            MotifBallColor detected = classify(r, g, b);

            telemetry.addData("Motif", motifName());
            telemetry.addData("Motif Step", (motifIndex + 1) + " / " + motif.length);
            telemetry.addData("Target Color", motif[motifIndex]);
            telemetry.addData("Detected Color", detected);
            telemetry.addData("Seeking Pocket", seekIndex + 1);

            switch (state) {

                case WAIT_FOR_Y:
                    if (gamepad2.y) {
                        motifIndex = 0;
                        firedOnce = false;
                        seekIndex = 0;
                        lastSeekSwitch = System.currentTimeMillis();
                        stableMatchCount = 0;
                        shooter.setVelocity(0);
                        state = MotifState.SPIN_UP;
                        resetTimer();
                    }
                    break;

                case SPIN_UP:
                    shooter.setVelocity(targetVelocity);
                    if (timePassed(1000)) {
                        state = MotifState.SEEKING;
                        seekIndex = 0;
                        lastSeekSwitch = System.currentTimeMillis();
                        stableMatchCount = 0;
                        resetTimer();
                    }
                    break;

                case SEEKING:

                    // Pocket cycling
                    if (System.currentTimeMillis() - lastSeekSwitch > SEEK_DWELL_MS) {
                        seekIndex = (seekIndex + 1) % 3;
                        lastSeekSwitch = System.currentTimeMillis();
                        stableMatchCount = 0;
                    }

                    switch (seekIndex) {
                        case 0: spindexer.setPosition(POS1); break;
                        case 1: spindexer.setPosition(POS2); break;
                        case 2: spindexer.setPosition(POS3); break;
                    }

                    // Stable match
                    if (detected == motif[motifIndex]) {
                        stableMatchCount++;
                    } else {
                        stableMatchCount = 0;
                    }

                    if (stableMatchCount >= REQUIRED_STABLE_MATCH) {
                        state = MotifState.ALIGNING;
                        stableMatchCount = 0;
                        resetTimer();
                    }

                    break;

                case ALIGNING:

                    // If wrong color appears, go back to SEEKING
                    if (detected != motif[motifIndex]) {
                        state = MotifState.SEEKING;
                        seekIndex = 0;
                        lastSeekSwitch = System.currentTimeMillis();
                        stableMatchCount = 0;
                        resetTimer();
                        break;
                    }

                    if (timePassed(1500)) {
                        state = MotifState.FIRING;
                        firedOnce = false;
                        resetTimer();
                    }
                    break;

                case FIRING:

                    // Final safety: re-check color
                    MotifBallColor confirm = classify(
                            colorSensor.red(),
                            colorSensor.green(),
                            colorSensor.blue()
                    );

                    if (confirm != motif[motifIndex]) {
                        telemetry.addLine("ABORT FIRE: Color mismatch");
                        state = MotifState.SEEKING;
                        seekIndex = 0;
                        stableMatchCount = 0;
                        resetTimer();
                        break;
                    }

                    if (!firedOnce) {
                        scooper.setPosition(SCOOP_EXTEND);
                        firedOnce = true;
                        resetTimer();
                    }

                    if (timePassed(500)) {
                        scooper.setPosition(SCOOP_RETRACT);
                        state = MotifState.ADVANCE;
                        resetTimer();
                    }
                    break;

                case ADVANCE:
                    motifIndex++;
                    if (motifIndex >= motif.length) {
                        state = MotifState.DONE;
                    } else {
                        state = MotifState.SEEKING;
                        seekIndex = 0;
                        lastSeekSwitch = System.currentTimeMillis();
                        stableMatchCount = 0;
                    }
                    resetTimer();
                    break;

                case DONE:
                    shooter.setVelocity(0);
                    break;

                case SELECTING:
                default:
                    break;
            }
        }
    }
}
