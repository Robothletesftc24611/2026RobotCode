package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "BLUE STATES", group = "TeleOp")
public class teleopv3blue extends OpMode {

    // === DRIVETRAIN ===
    private DcMotor lf, lb, rf, rb;

    // === INTAKE ===
    private DcMotorEx intake;

    // === GATE ===
    private Servo gate;
    private static final double GATE_CLOSED = 0.6;
    private static final double GATE_OPEN = 0.85;

    // === SHOOTER ===
    private DcMotorEx shooter1, shooter2;
    private final double kP = 30;
    private final double kF_base = 12;
    private final double nominalVoltage = 13.2;

    // === TURRET ===
    private DcMotorEx turretMotor;
    private Follower follower;

    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 144;

    private static final double TICKS_PER_DEG = 4.3;
    private static final double TURRET_MOUNT_OFFSET_DEG = 180.0;
    private static final int MIN_TICKS = -100;
    private static final int MAX_TICKS = 1000;

    private static final double MANUAL_TURRET_SPEED = 0.4;

    // Persistent manual mode flag
    private boolean manualTurretMode = false;

    // === SHOOTER FSM ===
    private enum ShootState { IDLE, SPINUP, FIRE, DONE }
    private ShootState shootState = ShootState.IDLE;
    private long fireStartTime = 0;

    @Override
    public void init() {

        // === DRIVETRAIN ===
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        // === INTAKE ===
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === GATE ===
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(GATE_CLOSED);

        // === SHOOTER ===
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotorEx.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setVelocityPIDFCoefficients(kP, 0, 0, kF_base);
        shooter2.setVelocityPIDFCoefficients(kP, 0, 0, kF_base);

        // === TURRET ===
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === PEDRO FOLLOWER ===
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(112.17, 68, Math.toRadians(180)));
        follower.startTeleopDrive();

        telemetry.addLine("TeleOp v4 Blue (Persistent Manual Turret) Initialized");
        telemetry.update();
    }

    // === SHOOTER HELPERS ===
    private double getCompensatedF() {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        return kF_base * (nominalVoltage / voltage);
    }

    private void setShooterVelocity(double targetVelo) {
        double f = getCompensatedF();
        shooter1.setVelocityPIDFCoefficients(kP, 0, 0, f);
        shooter2.setVelocityPIDFCoefficients(kP, 0, 0, f);
        shooter1.setVelocity(targetVelo);
        shooter2.setVelocity(targetVelo);
    }

    private void stopShooter() {
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }

    // === INTAKE HELPERS ===
    private void setIntakePower(double p) { intake.setPower(p); }
    private void stopIntake() { intake.setPower(0); }

    // === RANGE-BASED SHOOTER VELOCITY ===
    private double getVelocityFromRange(double distance) {
        return (distance < 100) ? 1400 : 1750;
    }

    // === RESET TURRET ENCODER AT CURRENT POSITION ===
    private void resetTurretEncoderHere() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        // === UPDATE LOCALIZATION ===
        follower.update();
        Pose pose = follower.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        // === DISTANCE TO TARGET ===
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;
        double distance = Math.hypot(dx, dy);

        // === DRIVETRAIN ===
        double axial   = gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        lf.setPower(fl);
        rf.setPower(fr);
        lb.setPower(bl);
        rb.setPower(br);

        // === MANUAL INTAKE ===
        if (shootState == ShootState.IDLE) {
            if (gamepad2.right_bumper) setIntakePower(-1.0);
            else if (gamepad2.a) setIntakePower(1.0);
            else stopIntake();
        }

        // === MANUAL GATE ===
        if (shootState == ShootState.IDLE) {
            if (gamepad2.x) gate.setPosition(GATE_OPEN);
            else if (gamepad2.y) gate.setPosition(GATE_CLOSED);
        }

        // === SHOOTER FSM ===
        if (gamepad2.x && shootState == ShootState.IDLE) shootState = ShootState.SPINUP;
        if (gamepad2.left_bumper) shootState = ShootState.DONE;

        switch (shootState) {

            case IDLE:
                stopShooter();
                gate.setPosition(GATE_CLOSED);
                break;

            case SPINUP:
                double targetVelocity = getVelocityFromRange(distance);
                setShooterVelocity(targetVelocity);
                gate.setPosition(GATE_CLOSED);
                stopIntake();

                boolean shooterReady =
                        Math.abs(shooter2.getVelocity() - targetVelocity) < 50;

                if (shooterReady) {
                    shootState = ShootState.FIRE;
                    fireStartTime = System.currentTimeMillis();
                }
                break;

            case FIRE:
                long elapsed = System.currentTimeMillis() - fireStartTime;

                gate.setPosition(GATE_OPEN);

                if (elapsed > 250) {
                    if (distance < 101) setIntakePower(-1.0);
                    else setIntakePower(-0.8);
                } else stopIntake();

                if (elapsed > 3000) shootState = ShootState.DONE;
                break;

            case DONE:
                stopShooter();
                gate.setPosition(GATE_CLOSED);
                stopIntake();
                shootState = ShootState.IDLE;
                break;
        }

        // === MANUAL TURRET CONTROL (D-PAD) ===
        boolean dpadLeft  = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;

        // Once manual mode is entered, it stays until Y is pressed
        if (dpadLeft || dpadRight) {
            manualTurretMode = true;
        }

        if (manualTurretMode) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (dpadLeft) {
                turretMotor.setPower(MANUAL_TURRET_SPEED);
            } else if (dpadRight) {
                turretMotor.setPower(-MANUAL_TURRET_SPEED);
            } else {
                turretMotor.setPower(0);
            }
        }

        // === DRIVER RE-ZEROS TURRET AND RE-ENABLES AUTO-AIM ===
        if (gamepad2.y) {
            resetTurretEncoderHere();
            manualTurretMode = false;
        }

        // === AUTO-AIM ONLY WHEN NOT IN MANUAL MODE ===
        if (!manualTurretMode) {

            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            double relativeAngleDeg = normalizeAngle(fieldAngleDeg - robotHeadingDeg);
            double turretAngleDeg = normalizeAngle(relativeAngleDeg + TURRET_MOUNT_OFFSET_DEG);

            int targetTicks = (int)(turretAngleDeg * TICKS_PER_DEG);
            targetTicks = clamp(targetTicks, MIN_TICKS, MAX_TICKS);
            turretMotor.setTargetPosition(targetTicks);

            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(0.7);
        }

        // === TELEMETRY ===
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("State", shootState);
        telemetry.addData("Distance (in)", distance);
        telemetry.addData("Chosen Velocity", getVelocityFromRange(distance));
        telemetry.addData("S1 Vel", shooter1.getVelocity());
        telemetry.addData("S2 Vel", shooter2.getVelocity());

        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Manual Mode", manualTurretMode);
        telemetry.addData("Turret Pos", turretMotor.getCurrentPosition());

        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}
