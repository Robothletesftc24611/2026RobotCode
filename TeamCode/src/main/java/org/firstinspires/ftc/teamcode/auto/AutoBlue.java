package org.firstinspires.ftc.teamcode.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BlueAuto")
public class AutoBlue extends OpMode {
    public DcMotor rightBackMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor leftFrontMotor;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(48, 96, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(48, 96, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(36, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(18, 84, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(36, 60, Math.toRadians(180));
    private final Pose pickup4Pose = new Pose(15, 60, Math.toRadians(180));


    @Override
    public void init() {

    }

    @Override
    public void start() {


    }
    @Override
    public void loop() {

    }




}
