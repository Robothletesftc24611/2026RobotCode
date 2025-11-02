package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name="Final TeleOp", group="Linear OpMode")
public class FinalTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake = null;
    private DcMotor Shooter = null;
    private Servo spindexer = null;

    private Servo scooper = null;

    private CRServo intakeServo = null;
    private Servo door = null;
    public Limelight3A limelight;
    public CRServo turretServo;

    double spindexerposition;

    double LIMELIGHT_OFFSET = 0.0;    // Calibrate this once and it stays fixed
    double DEADZONE = 1.0;            // Ignore small tx values to prevent twitching
    double MAX_POWER = 0.7;           // Limit max spin speed
    boolean buttonPressed = false;

    private final String[] motifs = {"GPP, PGP, PPG"};
    private int motifIndex = 0;
    private boolean aPressedLast = false;

    ColorSensor colorSensor;

    public void shootThreeBalls(){
        Shooter.setPower(-0.8);

        sleep(1000);

        spindexer.setPosition(0.0);
        sleep(1500);
        scooper.setPosition(0.5);
        sleep(1000);

        scooper.setPosition(0.0);

        spindexer.setPosition(0.4);
        sleep(1000);
        scooper.setPosition(0.5);
        sleep(1000);

        scooper.setPosition(0.0);

        spindexer.setPosition(0.85);
        sleep(1000);
        scooper.setPosition(0.5);
        sleep(1000);

        scooper.setPosition(0.0);

        Shooter.setPower(0.0);
    }


    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        intake = hardwareMap.get(DcMotor.class, "intake");
        spindexer = hardwareMap.get(Servo.class, "spindexer");
        scooper = hardwareMap.get(Servo.class, "scooper");
        door = hardwareMap.get(Servo.class, "door");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        scooper.setDirection(Servo.Direction.REVERSE);
        door.setPosition(0.0);

        while (!isStarted() && !isStopRequested()){
            if (gamepad1.a && !aPressedLast){
                motifIndex = (motifIndex + 1) % motifs.length;
            }
            aPressedLast = gamepad1.a;

            telemetry.addLine("Press A to cycle through motifs");
            telemetry.addData("Selected Motif", motifs[motifIndex]);
            telemetry.update();

            sleep(100);
        }

        double[] positions = {0.35, 0.8, 1};
        int currentPositionIndex = 0;

        boolean buttonPreviouslyPressed = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean buttonPressed = gamepad2.left_bumper;

            limelight.start();
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad2.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  -gamepad2.left_stick_x;
            double yaw     =  gamepad2.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            //limit the speed of the drivetrain, change to higher when more driver practice is done, now is 250rpm
            if (gamepad2.right_trigger > 0){
                frontLeftPower = Range.clip(frontLeftPower, -0.8, 0.8);
                frontRightPower = Range.clip(frontRightPower, -0.8, 0.8);
                backLeftPower = Range.clip(backLeftPower, -0.8, 0.8);
                backRightPower = Range.clip(backRightPower, -0.8, 0.8);
            }
            else{
                frontLeftPower = Range.clip(frontLeftPower, -0.55, 0.55);
                frontRightPower = Range.clip(frontRightPower, -0.55, 0.55);
                backLeftPower = Range.clip(backLeftPower, -0.55, 0.55);
                backRightPower = Range.clip(backRightPower, -0.55, 0.55);
            }

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Turret Code
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


            //intake code
            if (gamepad2.right_bumper){
                intake.setPower(-0.5);
                door.setPosition(0.05);
                intakeServo.setPower(1.0);
            }
            else {
                intake.setPower(0.0);
                door.setPosition(0.2);
                intakeServo.setPower(0.0);
            }

            if (buttonPressed && !buttonPreviouslyPressed){
                sleep(150);
                currentPositionIndex++;
                if (currentPositionIndex >= positions.length){
                    currentPositionIndex = 0;
                }
                spindexer.setPosition(positions[currentPositionIndex]);
            }

            buttonPreviouslyPressed = buttonPressed;

            //shooter code
            if(gamepad2.x){
                shootThreeBalls();
            }


            //scooper code
            if(gamepad2.y){
                scooper.setPosition(0.5);
            }
            else{
                scooper.setPosition(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Door Servo Position", door.getPosition());
            telemetry.addData("Shooter Speed",Shooter.getPower());
            telemetry.addData("Intake Speed", intake.getPower());
            telemetry.addData("Spindexer position", spindexer.getPosition());
            telemetry.update();
        }
    }
}
