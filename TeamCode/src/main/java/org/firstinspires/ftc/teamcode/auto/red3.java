package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "red 3 ball")


public class red3 extends OpMode {

    private DcMotor intake = null;
    private DcMotor Shooter = null;
    private Servo spindexer = null;

    private Servo scooper = null;
    private Servo door = null;
    public CRServo turretServo = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final double[] spindexerPositions = {0.0, 0.4, 0.85};
    private int shootState = 0;
    private int ballsShot = 0;
    private boolean shootingDone = false;

    private final Pose startPose = new Pose(111.9, 134.3, Math.toRadians(270));
    private final Pose scorePose = new Pose(106.4, 106.6, Math.toRadians(225));

    private final Pose finalPose = new Pose(124, 100, Math.toRadians(225));

    private Path scorePreload, park;

    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose,scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        park = new Path(new BezierLine(scorePose, finalPose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());
    }

    public void shootThreeUpdate(){
        switch(shootState){
            case 0:
                Shooter.setPower(-0.7);
                ballsShot = 0;
                spindexer.setPosition(spindexerPositions[ballsShot]);
                actionTimer.resetTimer();
                shootState = 10;
                break;
            case 10:
                if (actionTimer.getElapsedTimeSeconds() > 1){
                    actionTimer.resetTimer();
                    shootState = 1;
                }
                break;
            case 1:
                if (actionTimer.getElapsedTimeSeconds() > 0.75){
                    scooper.setPosition(0.5);
                    actionTimer.resetTimer();
                    shootState = 2;
                }
                break;
            case 2:
                if (actionTimer.getElapsedTimeSeconds() > 0.5){
                    scooper.setPosition(0.0);
                    ballsShot++;

                    if (ballsShot < spindexerPositions.length){
                        spindexer.setPosition(spindexerPositions[ballsShot]);
                        actionTimer.resetTimer();
                        shootState = 1;
                    } else{
                        Shooter.setPower(0.0);
                        shootingDone = true;
                        shootState = 3;
                    }
                }
        }
    }
    

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()){
                    shootState = 0;
                    ballsShot = 0;
                    shootThreeUpdate();
                    pathState = 2;
                }
                break;
            case 2:
                if (shootingDone){
                    follower.followPath(park);
                    pathState = 3;
                }
                break;
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        shootThreeUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("actionTimer", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("balls shot", ballsShot);
        telemetry.addData("shooter state", shootState);
        telemetry.addData("scooper position", scooper.getPosition());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        spindexer = hardwareMap.get(Servo.class, "spindexer");
        scooper = hardwareMap.get(Servo.class, "scooper");
        door = hardwareMap.get(Servo.class, "door");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");

        scooper.setDirection(Servo.Direction.REVERSE);
        door.setPosition(0.2);
        scooper.setPosition(0.0);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        pathState = 0;
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}


}
