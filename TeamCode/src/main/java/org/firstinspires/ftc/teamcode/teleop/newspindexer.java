package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "spindexer test", group = "Iterative OpMode")
public class newspindexer extends OpMode {
    private Servo servo;

    @Override
    public void init(){
        servo = hardwareMap.get(Servo.class, "spindexer");
    }

    @Override
    public void loop(){
        if (gamepad1.a){
            servo.setPosition(0.25);
        } else if (gamepad1.b){
            servo.setPosition(0.45);
        } else if (gamepad1.y){
            servo.setPosition(0.67);
        } else if (gamepad1.x){
            servo.setPosition(0.0);
        }
        telemetry.addData("servo position", servo.getPosition());
    }

}
