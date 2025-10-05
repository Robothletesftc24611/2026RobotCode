package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;


import java.util.List;


@TeleOp(name = "Turret Tracker CR", group = "Linear OpMode")
public class turretTrackerCR extends OpMode {


    Limelight3A limelight; //limelight camera
    CRServo turretServo; //servo for the turret


    // Tuning constants
    double LIMELIGHT_OFFSET = 0.0;    // Calibrate this once and it stays fixed
    double DEADZONE = 1.0;            // Ignore small tx values to prevent twitching
    double MAX_POWER = 0.3;           // Limit max spin speed


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");


        limelight.start(); //start the limelight vision portal
        turretServo.setPower(0.0); // Start with servo having 0 power
    }


    @Override
    public void start() {
        limelight.pipelineSwitch(2); // Pipeline 2 is for blue goal, 1 for red goal
    }


    @Override
    public void loop() {
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


        telemetry.update();
    }
}

