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

@Autonomous(name = "trial 3 red")

public class red3trial extends LinearOpMode{
    private DcMotor intake = null;
    private DcMotor Shooter = null;
    private Servo spindexer = null;

    private Servo scooper = null;
    private Servo door = null;
    public CRServo turretServo = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(110.8, 135.3, Math.toRadians(270));
    private final Pose scorePose = new Pose(104.4, 104.6, Math.toRadians(225));

    private Path scorePreload;

    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose,scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }

    @Override
    public void runOpMode(){
        intake = hardwareMap.get(DcMotor.class, "intake");
        spindexer = hardwareMap.get(Servo.class, "spindexer");
        scooper = hardwareMap.get(Servo.class, "scooper");
        door = hardwareMap.get(Servo.class, "door");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");

        scooper.setDirection(Servo.Direction.REVERSE);
        door.setPosition(0.2);
        scooper.setPosition(0.0);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            switch (pathState){
                case 0:
                    follower.followPath(scorePreload);
                    pathState = 1;
                    break;
            }
        }
    }

}
