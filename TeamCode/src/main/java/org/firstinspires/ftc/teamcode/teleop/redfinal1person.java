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
@TeleOp(name="FINAL RED TELE_OP - 1 person", group="Iterative OpMode")
public class redfinal1person extends OpMode {

    // Hardware
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor intake;
    private DcMotorEx shooter; // PIDF motor
    private Servo spindexer, scooper, door;
    private CRServo turretServo;
    private Limelight3A limelight;
    private ColorSensor colorSensor;
    private DigitalChannel ballSensor;
    private VoltageSensor batteryVoltage;

    // Shooter PIDF constants
    private static final double NOMINAL_VOLTAGE = 11.38;
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 23.0; // shooter motor feedforward

    private double targetVelocity = 0;
    private double distance = 0;

    //limelight setup
    private final double LIMELIGHT_OFFSET = -1.0;
    private final double DEADZONE = 1.0;
    private final double MAX_POWER = 1.0;

    // Shooter FSM
    private enum ShooterState { IDLE, SPINUP, INDEX_BALL, SCOOP, RESET, DONE }
    private ShooterState shooterState = ShooterState.IDLE;
    private ElapsedTime shooterTimer = new ElapsedTime();
    private int ballCount = 0;

    // Spindexer FSM
    private enum SpindexerState { IDLE, BALL_DETECTED, WAIT_DELAY, ADVANCE, DONE }
    private SpindexerState spindexerState = SpindexerState.IDLE;
    private ElapsedTime spindexerTimer = new ElapsedTime();
    private int currentPositionIndex = 0;
    private boolean buttonPreviouslyPressed = false;
    double[] positions = {0.1, 0.5, 1};

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        // Initialize shooter PIDF
        double compensatedF = kF * (NOMINAL_VOLTAGE / batteryVoltage.getVoltage());
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, compensatedF));

        // Servo defaults
        scooper.setDirection(Servo.Direction.REVERSE);
        spindexer.setPosition(0.1);
        scooper.setPosition(0.0);
        door.setPosition(0.0);

        telemetry.addData("Status", "Initialized with PIDF Shooter");
    }

    @Override
    public void loop() {
        driveControl();
        intakeControl();
        turretControl();
        manualTurretControl();
        shooterFSM();
        spindexerFSM();
        spindexerManualControl();
        telemetryOutput();
    }

    // --------------------- Drive ---------------------
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


    // --------------------- Intake ---------------------
    private void intakeControl() {
        if (gamepad1.right_bumper) {
            intake.setPower(-0.5);
            door.setPosition(-0.5);
        }
        else if (gamepad1.a){
            intake.setPower(0.5);
            door.setPosition(-0.5);
        } else {
            intake.setPower(0);
            door.setPosition(0.2);
        }
    }

    // --------------------- Turret ---------------------
    private void turretControl() {
        LLResult llResult = limelight.getLatestResult(); //get latest result of what the camera sees
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults(); //sort them into a list


        if (!fiducials.isEmpty()) {
            int tagId = fiducials.get(0).getFiducialId(); //get the most recent reading from the list
            double tx = llResult.getTx(); //get the x axis offset of the most recent reading to center the turret properly
            double correctedTx = tx - LIMELIGHT_OFFSET;


            telemetry.addData("AprilTag ID", tagId);  //display values so it is easier to debug
            telemetry.addData("Raw tx", "%.2f", tx);
            telemetry.addData("Corrected tx", "%.2f", correctedTx);


            if (Math.abs(correctedTx) > DEADZONE) { //only spin if it is needed (for small changes (<1 degree) there is no point of spinning)
                double power = -correctedTx / 30.0; // make sure that the range for power is always -1 to 1
                power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power)); //make sure power never goes over 0.3


                turretServo.setPower(power); //set power value to the servo
                telemetry.addData("Turret Status", "Spinning");
                telemetry.addData("Servo Power", "%.2f", power);
            } else {
                turretServo.setPower(0.0); // Stop when centered
                telemetry.addData("Turret Status", "Centered");
            }
        } else {
            turretServo.setPower(0.0); // Stop if no tag
            telemetry.addData("AprilTag", "Not Found");
            telemetry.addData("Turret Status", "Holding");
        }
    }

    private void manualTurretControl(){
        if (gamepad1.dpad_left) turretServo.setPower(1.0);
        else if (gamepad1.dpad_right) turretServo.setPower(-1.0);
    }

    // --------------------- Shooter FSM ---------------------
    private void shooterFSM() {
        LLResult llResult = limelight.getLatestResult();
        distance = getDistanceFromTag(llResult.getTa());
        if (distance > 170){
            targetVelocity = 1500;
        } else {
            targetVelocity = 1250; // default if no target
        }

        if (gamepad1.x && shooterState == ShooterState.IDLE) {
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

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Distance", distance);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", shooter.getVelocity());
        telemetry.addData("Error", targetVelocity - shooter.getVelocity());
    }

    private double getDistanceFromTag(double ta) {
        return (-33.74145 * ta) + 194.923;
    }

    // --------------------- Spindexer FSM ---------------------
    private void spindexerFSM() {
        boolean ballDetected = !ballSensor.getState(); // HIGH/LOW depends on your sensor
        switch (spindexerState) {
            case IDLE:
                if (ballDetected) spindexerState = SpindexerState.BALL_DETECTED;
                break;
            case BALL_DETECTED:
                spindexerTimer.reset();
                spindexerState = SpindexerState.WAIT_DELAY;
                break;
            case WAIT_DELAY:
                if (spindexerTimer.seconds() > 0.1) spindexerState = SpindexerState.ADVANCE;
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
        boolean buttonPressed = gamepad1.left_bumper;
        if (buttonPressed && !buttonPreviouslyPressed){
            currentPositionIndex++;
            if (currentPositionIndex >= positions.length) currentPositionIndex = 0;
            spindexer.setPosition(positions[currentPositionIndex]);
        }
        buttonPreviouslyPressed = buttonPressed;
    }

    // --------------------- Telemetry ---------------------
    private void telemetryOutput() {
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Spindexer Position", spindexer.getPosition());
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }
}
