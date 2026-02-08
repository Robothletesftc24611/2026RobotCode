package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp (name = "Intake Testing", group = "Iterative Opmode")
public class intaketest extends OpMode {
    private DcMotor intake;

    @Override
    public void init(){
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop(){
        if (gamepad1.a){
            intake.setPower(-0.5);
        } else if(gamepad1.b){
            intake.setPower(-0.7);
        } else{
            intake.setPower(0.0);
        }
        telemetry.addData("Intake power:", intake.getPower());
        telemetry.update();
    }

}
