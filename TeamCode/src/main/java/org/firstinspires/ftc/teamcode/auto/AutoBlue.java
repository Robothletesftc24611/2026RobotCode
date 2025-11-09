package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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
    private final Pose startPose = new Pose(26.418, 130.867, Math.toRadians(324));
    private final Pose obelisk = new Pose(60.624, 129.951, Math.toRadians(270));
    private final Pose scorePose = new Pose(57.875, 84.445, Math.toRadians(324));
    private final Pose pickup1_start = new Pose(45.353, 84.293, Math.toRadians(0));
    private final Pose pickup1_score = new Pose(17.2555, 84.293, Math.toRadians(0));
    private final Pose pickup2_start = new Pose(45.353, 60.471, Math.toRadians(0));
    private final Pose pickup2_score = new Pose(17.2555, 60.471, Math.toRadians(0));


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    private PathChain motif, scorePreload, pos_1, pick_1, score_1, pos_2, pick_2, score_2;
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

        pos_1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose , pickup1_start))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1_start.getHeading())
                .build();
        pick_1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1_start , pickup1_score))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1_start.getHeading())
                .build();
        score_1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1_score , scorePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1_start.getHeading())
                .build();


        pos_2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose , pickup2_start))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1_start.getHeading())
                .build();
        pick_2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2_start , pickup2_score))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1_start.getHeading())
                .build();
        score_2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2_score , scorePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1_start.getHeading())
                .build();



    }
    public void autonomousPathUpdate() {

        //private PathChain motif, scorePreload, pos_1, pick_1, score_1, pos_2, pick_2, score_2;
        switch (pathState) {
            case 0:
                follower.followPath(motif);
                setPathState(1);
                break;
            case 1:

                if(!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(pos_1);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(pick_1);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(score_1);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(pos_2);
                    setPathState(6);
                }
                break;
            case 6:
               if(!follower.isBusy()) {
                    follower.followPath(pick_2);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(score_2);
                    setPathState(-1);
                }
                break;
        }
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
