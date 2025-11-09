package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.88)
            .forwardZeroPowerAcceleration(-41.31)
            .lateralZeroPowerAcceleration(-84.76)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0.0,0.01,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0.0,0.02,0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0.0,0.00002, 0.4, 0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftFrontMotorName("front_left_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.39)
            .yVelocity(72.76);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.002085)
            .strafeTicksToInches(0.002478)
            .turnTicksToInches(0.001763)
            .leftPodY(6)
            .rightPodY(1)
            .strafePodX(6.75)
            .leftEncoder_HardwareMapName("front_right_drive")
            .rightEncoder_HardwareMapName("back_right_drive")//back_right_drive
            .strafeEncoder_HardwareMapName("front_left_drive")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
