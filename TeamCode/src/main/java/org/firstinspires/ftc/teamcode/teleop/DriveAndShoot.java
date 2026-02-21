package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name = "DriveAndShoot", group = "TeleOp")
public class DriveAndShoot extends OpMode {

    // ---------------- HARDWARE ----------------
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor intake;
    private DcMotorEx shooter1, shooter2;
    private Servo gate;

    // ---------------- TIMERS ----------------
    private final ElapsedTime shootTimer = new ElapsedTime();

    // ---------------- SHOOTER FSM ----------------
    private enum ShootState { IDLE, SPINUP, FEED, SPINDOWN }
    private ShootState shootState = ShootState.IDLE;

    // ---------------- CONSTANTS ----------------

    // Drivetrain speed limit
    private static final double DRIVE_MAX_SPEED = 0.9;   // 60% power cap

    // PIDF for shooter
    private static final double kP = 50.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 30.0;

    // Shooter velocity target (ticks/sec)
    private static final double SHOOTER_TARGET_VELOCITY = 1500;

    // Timing (seconds)
    private static final double SPINUP_TIME   = 3.0;
    private static final double FEED_TIME     = 2.0;
    private static final double SPINDOWN_TIME = 1.0;

    // Gate positions
    private static final double GATE_CLOSED = 0.5;
    private static final double GATE_OPEN   = 1.0;

    // Intake
    private static final double INTAKE_POWER = -1.0;

    // ---------------- INIT ----------------
    @Override
    public void init() {

        // Map hardware
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "lb");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "lf");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive  = hardwareMap.get(DcMotor.class, "rb");

        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        // Drive directions
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Shooter directions
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Shooter encoders + PIDF
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Gate default

        telemetry.addLine("Initialized");
    }

    // ---------------- LOOP ----------------
    @Override
    public void loop() {
        drive();
        handleManualIntake();
        handleShootTrigger();
        updateShootFSM();
        sendTelemetry();
    }

    // ---------------- DRIVE ----------------
    private void drive() {
        double axial   = gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        // Normalize
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));

        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        // Apply speed limit
        fl = Range.clip(fl, -DRIVE_MAX_SPEED, DRIVE_MAX_SPEED);
        fr = Range.clip(fr, -DRIVE_MAX_SPEED, DRIVE_MAX_SPEED);
        bl = Range.clip(bl, -DRIVE_MAX_SPEED, DRIVE_MAX_SPEED);
        br = Range.clip(br, -DRIVE_MAX_SPEED, DRIVE_MAX_SPEED);

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    // ---------------- MANUAL INTAKE ----------------
    private void handleManualIntake() {
        if (shootState == ShootState.IDLE) {
            if (gamepad2.right_bumper) intake.setPower(-0.8);
            else if (gamepad2.a) intake.setPower(0.5);
            else intake.setPower(0.0);
        }
    }

    // ---------------- SHOOT TRIGGER ----------------
    private void handleShootTrigger() {
        if (gamepad2.x && shootState == ShootState.IDLE) {
            shootState = ShootState.SPINUP;
            shootTimer.reset();
        }
    }

    // ---------------- SHOOT FSM ----------------
    private void updateShootFSM() {
        switch (shootState) {

            case IDLE:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                break;

            case SPINUP:
                shooter1.setVelocity(SHOOTER_TARGET_VELOCITY);
                shooter2.setVelocity(SHOOTER_TARGET_VELOCITY);

                if (shootTimer.seconds() >= SPINUP_TIME) {
                    shootState = ShootState.FEED;
                    shootTimer.reset();
                    intake.setPower(INTAKE_POWER);
                }
                break;

            case FEED:
                if (shootTimer.seconds() >= FEED_TIME) {
                    shootState = ShootState.SPINDOWN;
                    shootTimer.reset();
                    intake.setPower(0.0);
                }
                break;

            case SPINDOWN:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);

                if (shootTimer.seconds() >= SPINDOWN_TIME) {
                    shootState = ShootState.IDLE;
                }
                break;
        }
    }

    // ---------------- TELEMETRY ----------------
    private void sendTelemetry() {

        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Max Speed", DRIVE_MAX_SPEED);
        telemetry.addData("FL", frontLeftDrive.getPower());
        telemetry.addData("FR", frontRightDrive.getPower());
        telemetry.addData("BL", backLeftDrive.getPower());
        telemetry.addData("BR", backRightDrive.getPower());

        telemetry.addLine("\n=== SHOOTER ===");
        double vel1 = shooter1.getVelocity();
        double vel2 = shooter2.getVelocity();

        telemetry.addData("State", shootState);
        telemetry.addData("Target Vel", SHOOTER_TARGET_VELOCITY);
        telemetry.addData("Vel1", vel1);
        telemetry.addData("Vel2", vel2);
        telemetry.addData("Err1", SHOOTER_TARGET_VELOCITY - vel1);
        telemetry.addData("Err2", SHOOTER_TARGET_VELOCITY - vel2);

        PIDFCoefficients pidf = shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("PIDF", "P:%.1f I:%.1f D:%.1f F:%.1f", pidf.p, pidf.i, pidf.d, pidf.f);

        telemetry.addLine("\n=== SHOOT TIMING ===");
        telemetry.addData("Timer", shootTimer.seconds());

        telemetry.addLine("\n=== INTAKE ===");
        telemetry.addData("Power", intake.getPower());

        telemetry.update();
    }
}
