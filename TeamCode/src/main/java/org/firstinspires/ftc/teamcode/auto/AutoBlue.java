package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous(name = "BlueAuto")
public class AutoBlue extends OpMode {
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake = null;
    private DcMotor Shooter = null;
    private Servo spindexer = null;

    private Servo scooper = null;
    private Servo door = null;
    public Limelight3A limelight;
    public CRServo turretServo;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // Define robot poses
    private final Pose startPose = new Pose(26.418, 130.867, Math.toRadians(144));
    private final Pose obelisk = new Pose(60.624, 129.951, Math.toRadians(180));
    private final Pose scorePose = new Pose(57.875, 84.445, Math.toRadians(135));
    private final Pose pickup1_start = new Pose(45.353, 84.293, Math.toRadians(180));
    private final Pose pickup1_score = new Pose(17.2555, 84.293, Math.toRadians(180));
    private final Pose pickup2_start = new Pose(45.353, 60.471, Math.toRadians(180));
    private final Pose pickup2_score = new Pose(17.2555, 60.471, Math.toRadians(180));


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    private PathChain motif, scorePreload, pos_,score1;
    public void buildPaths() {
        // Grab & Collect Pickup 1
        motif = follower.pathBuilder()
                .addPath(new BezierLine(startPose, obelisk))
                .setLinearHeadingInterpolation(scorePose.getHeading(), obelisk.getHeading())
                .build();
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(obelisk , scorePose))
                .setLinearHeadingInterpolation(obelisk.getHeading(),scorePose.getHeading())
                .build();



    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        follower.followPath(scorePreload);
    }

    @Override
    public void loop() {
        follower.update();
    }
}
