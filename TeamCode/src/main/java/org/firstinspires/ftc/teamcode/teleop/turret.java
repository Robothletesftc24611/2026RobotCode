package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@TeleOp(name = "Turret Tracking Test", group = "TeleOp")
public class turret extends OpMode {

    // === Hardware ===
    private DcMotorEx turretMotor;
    private Follower follower;
    private Pose pose;

    // === Target Position (Field Coordinates) ===
    private static final double TARGET_X = 144;
    private static final double TARGET_Y = 144;

    // === Turret Mechanical Constants ===
    private static final double MOTOR_TICKS_PER_REV = 383.6;   // 312 RPM Yellow Jacket
    private static final double GEAR_RATIO = 134.0 / 45.0;     // 45T → 134T
    private static final double TURRET_TICKS_PER_REV = MOTOR_TICKS_PER_REV * GEAR_RATIO;
    private static final double TICKS_PER_DEG = 4.5;

    // Turret zero points backwards
    private static final double TURRET_MOUNT_OFFSET_DEG = 180.0;

    // Soft limits (prevent wire destruction)
    private static final int MIN_TICKS = -100;
    private static final int MAX_TICKS = 1000;

    @Override
    public void init() {

        // === Initialize Turret Motor ===
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(0.7);

        // === Initialize Localization ===
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.startTeleopDrive();

        telemetry.addLine("Turret Tracking Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // === Update Robot Pose ===
        follower.update();
        pose = follower.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        // === Compute Field Angle to Target ===
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // === Convert to Robot-Relative Angle ===
        double relativeAngleDeg = fieldAngleDeg - robotHeadingDeg;
        relativeAngleDeg = normalizeAngle(relativeAngleDeg);

        // === Apply Turret Mount Offset ===
        double turretAngleDeg = relativeAngleDeg + TURRET_MOUNT_OFFSET_DEG;
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        // === Convert to Encoder Ticks ===
        int targetTicks = (int)(turretAngleDeg * TICKS_PER_DEG);

        // === Apply Soft Limits ===
        targetTicks = clamp(targetTicks, MIN_TICKS, MAX_TICKS);

        turretMotor.setTargetPosition(targetTicks);

        // === Telemetry ===
        telemetry.addLine("=== POSE ===");
        telemetry.addData("X", robotX);
        telemetry.addData("Y", robotY);
        telemetry.addData("Heading", robotHeadingDeg);

        telemetry.addLine("\n=== TURRET ===");
        telemetry.addData("Field Angle", fieldAngleDeg);
        telemetry.addData("Relative Angle", relativeAngleDeg);
        telemetry.addData("Turret Angle", turretAngleDeg);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Current Ticks", turretMotor.getCurrentPosition());

        telemetry.update();
    }

    // === Utility: Normalize angle to [-180, 180] ===
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // === Utility: Clamp value ===
    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}
