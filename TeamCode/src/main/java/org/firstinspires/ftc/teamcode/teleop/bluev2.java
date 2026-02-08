package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import java.util.List;

@TeleOp(name = "bluev2", group = "Iterative Opmode")
public class bluev2 extends OpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    private DcMotor intake;
    private DcMotorEx shooter1, shooter2;

    private Servo spindexer, scooper, door;
    private CRServo turretServo;
    private Limelight3A limelight;
    private ColorSensor colorSensor;
    private VoltageSensor batteryVoltage;

    private boolean lastRB = false;


    private boolean lastY = false;


    private final double LIMELIGHT_OFFSET = 0.0;
    private final double DEADZONE = 1.0;
    private final double MAX_POWER = 1.0;

    private enum BallColor { PURPLE, GREEN, UNKNOWN }

    // Slots
    private BallColor slotOne = BallColor.UNKNOWN;
    private BallColor slotTwo = BallColor.UNKNOWN;
    private BallColor slotThree = BallColor.UNKNOWN;

    // Tracks which slot we are filling next
    private int currentSlot = 1;

    // For rising-edge detection
    private BallColor lastDetectedColor = BallColor.UNKNOWN;

    // Shooter flag (you will set this to true later when shooting happens)
    private boolean hasShot = false;

    // Spindexer positions (adjust these to your robot)
    private final double SLOT1_POS = 0.13;
    private final double SLOT2_POS = 0.33;
    private final double SLOT3_POS = 0.55;
    private final double BLOCKED_NEUTRAL_POS = 0.67;   // No ball can enter

    private int stableCount = 0;
    private final int REQUIRED_STABLE_FRAMES = 2;  // 2 loops = ~40ms

    // PIDF constants
    private static final double NOMINAL_VOLTAGE = 11.91;
    private static final double kP = 12;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 13;   // tune this first

    private enum ShooterState {
        IDLE,
        SPINUP,
        ALIGN_SLOT,
        FIRE,
        RESET,
        DONE
    }

    private enum MotifState {
        OFF,
        SPINUP,
        WAIT_FOR_COLOR,
        ALIGN,
        FIRE,
        RESET
    }
    private MotifState motifState = MotifState.OFF;
    private BallColor requestedColor = BallColor.UNKNOWN;
    private int requestedSlot = -1;






    private ShooterState shooterState = ShooterState.IDLE;
    private ElapsedTime shooterTimer = new ElapsedTime();

    private int currentShootSlot = 1;
    private double targetVelocity = 1200;
    private double distance = 0;

    private double SLOT1_POSs = 0.25;
    private double SLOT2_POSs = 0.45;
    private double SLOT3_POSs = 0.67;

    private enum IntakeState {
        WAIT_FOR_BALL,
        DETECTING,
        LOCKED_IN,
        ADVANCING,
        FULL
    }
    private double spinup = 2.0;

    private IntakeState intakeState = IntakeState.WAIT_FOR_BALL;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private BallColor detectedColor = BallColor.UNKNOWN;



    @Override
    public void init(){
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");


        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        spindexer = hardwareMap.get(Servo.class, "spindexer");
        scooper = hardwareMap.get(Servo.class, "kicker");
        door = hardwareMap.get(Servo.class, "door");
        turretServo = hardwareMap.get(CRServo.class, "turret");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        door.setPosition(1.0);

        slotOne = BallColor.UNKNOWN;
        slotTwo = BallColor.UNKNOWN;
        slotThree = BallColor.UNKNOWN;
        currentSlot = 1;
        lastDetectedColor = BallColor.UNKNOWN;
        hasShot = false;

        // Start spindexer at slot 1 loading position
        spindexer.setPosition(SLOT1_POS);

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        double compensatedF = kF * (NOMINAL_VOLTAGE / batteryVoltage.getVoltage());

        shooter1.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, compensatedF)
        );

        shooter2.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, compensatedF)
        );

        scooper.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {

        double voltage = batteryVoltage.getVoltage();
        double dynamicF = kF * (NOMINAL_VOLTAGE / voltage);

        shooter1.setVelocityPIDFCoefficients(kP, kI, kD, dynamicF);
        shooter2.setVelocityPIDFCoefficients(kP, kI, kD, dynamicF);

        boolean yPressed = gamepad2.y;
        if (yPressed && !lastY) {
            if (motifState == MotifState.OFF) {
                motifState = MotifState.SPINUP;
                shooterTimer.reset();
            } else {
                motifState = MotifState.OFF;
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                scooper.setPosition(0.0);
                spindexer.setPosition(SLOT1_POS);
                currentSlot = 1;
            }
        }
        lastY = yPressed;

        // SHOOTER FSM
        if (motifState != MotifState.OFF) {
            motifShootFSM();
        } else {
            shooterFSM();
        }

        // ======== INTAKE + SPINDEXER LOGIC (CORRECT ORDER) ========
        if (shooterBusy()) {
            // Freeze auto-intake during shooting
            intakeState = IntakeState.WAIT_FOR_BALL;
            intakeTimer.reset();
        }
        else if (isManualSpinInput()) {
            // Manual override
            manualSpindexerControl();
        }
        else {
            // Auto intake
            handleColorIntake();
        }

        // ======== DRIVE + OTHER CONTROLS ========
        driveControl();
        intakeControl();
        turretControl();
        manualTurretControl();

        // TELEMETRY
        telemetry.addData("Slot 1", slotOne);
        telemetry.addData("Slot 2", slotTwo);
        telemetry.addData("Slot 3", slotThree);
        telemetry.addData("Current Slot", currentSlot);

        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);

        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Sat", hsv[1]);
        telemetry.addData("Val", hsv[2]);
        telemetry.addData("Detected Color", lastDetectedColor);
        telemetry.addData("shooter velocity", shooter2.getVelocity());

        telemetry.update();
    }



    private void motifShootFSM() {

        switch (motifState) {

            // ----------------------------------------------------
            // OFF — wheels off, waiting for driver to enable mode
            // ----------------------------------------------------
            case OFF:
                break;

            // ----------------------------------------------------
            // SPINUP — spin wheels once at start of endgame
            // ----------------------------------------------------
            case SPINUP:
                shooter1.setVelocity(1000);
                shooter2.setVelocity(1000);

                if (shooterTimer.seconds() > spinup) {
                    motifState = MotifState.WAIT_FOR_COLOR;
                }
                break;

            // ----------------------------------------------------
            // WAIT_FOR_COLOR — driver chooses next ball to shoot
            // ----------------------------------------------------
            case WAIT_FOR_COLOR:

                if (gamepad2.x) requestedColor = BallColor.PURPLE;
                else if (gamepad2.b) requestedColor = BallColor.GREEN;
                else break; // no request yet

                // Find which slot contains that co
                // lor
                requestedSlot = findSlotWithColor(requestedColor);

                if (requestedSlot == -1) {
                    // No ball of that color
                    requestedColor = BallColor.UNKNOWN;
                    break;
                }

                // Move spindexer to that slot
                if (requestedSlot == 1) spindexer.setPosition(SLOT2_POSs);
                if (requestedSlot == 2) spindexer.setPosition(SLOT3_POSs);
                if (requestedSlot == 3) spindexer.setPosition(SLOT1_POSs);

                shooterTimer.reset();
                motifState = MotifState.ALIGN;
                break;

            // ----------------------------------------------------
            // ALIGN — wait for servo to settle
            // ----------------------------------------------------
            case ALIGN:
                if (shooterTimer.seconds() > 1.0) {
                    scooper.setPosition(0.35);
                    shooterTimer.reset();
                    motifState = MotifState.FIRE;
                }
                break;

            // ----------------------------------------------------
            // FIRE — kick ball
            // ----------------------------------------------------
            case FIRE:
                if (shooterTimer.seconds() > 0.3) {
                    scooper.setPosition(0.0);
                    shooterTimer.reset();
                    motifState = MotifState.RESET;
                }
                break;

            // ----------------------------------------------------
            // RESET — clear slot, return to WAIT_FOR_COLOR
            // ----------------------------------------------------
            case RESET:

                // Clear the slot we just shot
                if (requestedSlot == 1) slotOne = BallColor.UNKNOWN;
                if (requestedSlot == 2) slotTwo = BallColor.UNKNOWN;
                if (requestedSlot == 3) slotThree = BallColor.UNKNOWN;

                requestedColor = BallColor.UNKNOWN;
                requestedSlot = -1;

                if (shooterTimer.seconds() > 0.3) {
                    motifState = MotifState.WAIT_FOR_COLOR;
                }
                break;
        }
    }

    private boolean isManualSpinInput() {
        return gamepad2.left_bumper;
    }


    private void manualSpindexerControl() {

        intakeState = IntakeState.WAIT_FOR_BALL;

        boolean rb = gamepad2.left_bumper;

        if (rb && !lastRB) {   // rising edge → rotate once

            if (currentSlot == 1) {
                spindexer.setPosition(SLOT2_POS);
                currentSlot = 2;

            } else if (currentSlot == 2) {
                spindexer.setPosition(SLOT3_POS);
                currentSlot = 3;

            } else if (currentSlot == 3) {
                spindexer.setPosition(SLOT1_POS);
                currentSlot = 1;
            }
        }

        lastRB = rb;
    }


    private int findSlotWithColor(BallColor color) {
        if (slotOne == color) return 1;
        if (slotTwo == color) return 2;
        if (slotThree == color) return 3;
        return -1;
    }



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

        double speedLimit = (gamepad1.right_trigger > 0) ? 1.0 : 1.0;
        fl = Range.clip(fl, -speedLimit, speedLimit);
        fr = Range.clip(fr, -speedLimit, speedLimit);
        bl = Range.clip(bl, -speedLimit, speedLimit);
        br = Range.clip(br, -speedLimit, speedLimit);

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    private void intakeControl(){
        if (gamepad2.right_bumper){
            intake.setPower(-1.0);
            door.setPosition(0.7);
        } else if (gamepad2.a){
            intake.setPower(0.3);
            door.setPosition(0.7);
        } else {
            intake.setPower(0.0);
            door.setPosition(1.0);
        }
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
    private void manualTurretControl(){
        if (gamepad2.dpad_left) turretServo.setPower(1.0);
        else if (gamepad2.dpad_right) turretServo.setPower(-1.0);
    }

    private BallColor detectBallColorHSV() {
        float[] hsv = new float[3];

        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        android.graphics.Color.RGBToHSV(r, g, b, hsv);

        float h = hsv[0];
        float s = hsv[1];
        float v = hsv[2];

        // Ignore readings when saturation or value are too low
        if (v < 0.7) {
            return BallColor.UNKNOWN;
        }

        // Tuned hue ranges
        boolean isPurple = (h > 190 && h < 250);
        boolean isGreen  = (h > 130 && h < 180);

        if (isPurple) return BallColor.PURPLE;
        if (isGreen)  return BallColor.GREEN;

        return BallColor.UNKNOWN;
    }


    private void handleColorIntake() {
        BallColor currentColor = detectBallColorHSV();

        switch (intakeState) {

            // ----------------------------------------------------
            // 1. WAIT FOR BALL
            // ----------------------------------------------------
            case WAIT_FOR_BALL:

                // If all 3 slots filled → lock out
                if (currentSlot > 3) {
                    intakeState = IntakeState.FULL;
                    break;
                }

                if (currentColor != BallColor.UNKNOWN) {
                    detectedColor = currentColor;
                    stableCount = 1;
                    intakeState = IntakeState.DETECTING;
                }
                break;

            // ----------------------------------------------------
            // 2. DETECTING (debounce)
            // ----------------------------------------------------
            case DETECTING:

                // If servo is moving, DO NOT detect
                if (intakeTimer.milliseconds() < 350) break;

                if (currentColor == detectedColor) {
                    stableCount++;
                } else {
                    intakeState = IntakeState.WAIT_FOR_BALL;
                    break;
                }

                if (stableCount >= REQUIRED_STABLE_FRAMES) {
                    intakeState = IntakeState.LOCKED_IN;
                }
                break;

            // ----------------------------------------------------
            // 3. LOCKED IN (assign slot + move servo)
            // ----------------------------------------------------
            case LOCKED_IN:

                if (currentSlot == 1) {
                    slotOne = detectedColor;
                    spindexer.setPosition(SLOT2_POS);
                    currentSlot = 2;

                } else if (currentSlot == 2) {
                    slotTwo = detectedColor;
                    spindexer.setPosition(SLOT3_POS);
                    currentSlot = 3;

                } else if (currentSlot == 3) {
                    slotThree = detectedColor;

                    // Move to blocked position
                    spindexer.setPosition(SLOT1_POS);
                    currentSlot = 1; // mark FULL
                }

                intakeTimer.reset();
                intakeState = IntakeState.ADVANCING;
                break;

            // ----------------------------------------------------
            // 4. ADVANCING (wait for servo to finish)
            // ----------------------------------------------------
            case ADVANCING:
                if (intakeTimer.milliseconds() > 350) {
                    if (currentSlot > 3) {
                        intakeState = IntakeState.FULL;
                    } else {
                        intakeState = IntakeState.WAIT_FOR_BALL;
                    }
                }
                break;

            // ----------------------------------------------------
            // 5. FULL (ignore all colors)
            // ----------------------------------------------------
            case FULL:
                gamepad1.rumble(250);
                gamepad2.rumble(250);
                // Do nothing — intake is locked out
                break;
        }
    }
    private boolean shooterBusy() {
        return shooterState != ShooterState.IDLE || motifState != MotifState.OFF;
    }



    private double getDistanceFromTag(double ta) {
        return (-33.74145 * ta) + 194.923;
    }

    // ============================================================
//  SHOOTER FSM (ported from redfinal → adapted for redv2)
// ============================================================
    private void shooterFSM() {

        // Compute distance + target velocity
        LLResult llResult = limelight.getLatestResult();
        distance = getDistanceFromTag(llResult.getTa());

        if (distance > 170){
            targetVelocity = 1225;
            spinup = 2.5;
        } else {
            targetVelocity = 1100;
            spinup = 2.0;
        }

        // Start sequence
        if (gamepad1.x && shooterState == ShooterState.IDLE) {
            shooterState = ShooterState.SPINUP;
            shooterTimer.reset();
            currentShootSlot = 1;
            hasShot = false;
        }


        switch (shooterState) {

            // ----------------------------------------------------
            // SPINUP
            // ----------------------------------------------------
            case SPINUP:
                shooter1.setVelocity(targetVelocity);
                shooter2.setVelocity(targetVelocity);
                spindexer.setPosition(SLOT1_POSs);
                double error = targetVelocity - shooter2.getVelocity();
                colorSensor.close();

                if (shooterTimer.seconds() > spinup) {
                    shooterState = ShooterState.ALIGN_SLOT;
                    shooterTimer.reset();
                }
                break;

            // ----------------------------------------------------
            // ALIGN SLOT
            // ----------------------------------------------------
            case ALIGN_SLOT:

                if (currentShootSlot == 1) spindexer.setPosition(SLOT1_POSs);
                else if (currentShootSlot == 2) spindexer.setPosition(SLOT2_POSs);
                else if (currentShootSlot == 3) spindexer.setPosition(SLOT3_POSs);

                if (shooterTimer.seconds() > 0.3) {
                    shooterState = ShooterState.FIRE;
                    shooterTimer.reset();
                }
                break;

            // ----------------------------------------------------
            // FIRE
            // ----------------------------------------------------
            case FIRE:
                scooper.setPosition(0.35);   // extend kicker

                if (shooterTimer.seconds() > 0.3) {
                    shooterState = ShooterState.RESET;
                    shooterTimer.reset();
                }
                break;

            // ----------------------------------------------------
            // RESET
            // ----------------------------------------------------
            case RESET:
                scooper.setPosition(0.0);   // retract kicker

                // Only run this once when entering RESET
                if (shooterTimer.seconds() == 0) {
                    // mark slot as fired
                    if (currentShootSlot == 1) slotOne = BallColor.UNKNOWN;
                    else if (currentShootSlot == 2) slotTwo = BallColor.UNKNOWN;
                    else if (currentShootSlot == 3) slotThree = BallColor.UNKNOWN;
                }

                // Wait 1 second before rotating
                if (shooterTimer.seconds() > 0.4) {

                    currentShootSlot++;

                    if (currentShootSlot > 3) {
                        shooterState = ShooterState.DONE;
                    } else {
                        shooterState = ShooterState.ALIGN_SLOT;
                    }

                    shooterTimer.reset();   // reset ONLY when leaving RESET
                }
                break;


            // ----------------------------------------------------
            // DONE
            // ----------------------------------------------------
            case DONE:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);

                gamepad1.rumble(250);
                gamepad2.rumble(250);


                hasShot = true;

                // Fully reset intake + spindexer
                resetIntakeFSM();

                shooterState = ShooterState.IDLE;
                break;


            case IDLE:
            default:
                break;
        }
    }

    private void resetIntakeFSM() {
        slotOne = BallColor.UNKNOWN;
        slotTwo = BallColor.UNKNOWN;
        slotThree = BallColor.UNKNOWN;

        currentSlot = 1;
        intakeState = IntakeState.WAIT_FOR_BALL;

        stableCount = 0;
        detectedColor = BallColor.UNKNOWN;
        lastDetectedColor = BallColor.UNKNOWN;

        spindexer.setPosition(SLOT1_POS);
        intakeTimer.reset();
    }








}
