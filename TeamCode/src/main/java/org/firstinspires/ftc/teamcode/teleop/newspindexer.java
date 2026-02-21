package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "gate test", group = "Iterative OpMode")
public class newspindexer extends OpMode {
    private Servo gate;

    @Override
    public void init(){
        gate = hardwareMap.get(Servo.class, "gate");
    }

    @Override
    public void loop(){
        if (gamepad1.a){
            gate.setPosition(0.0);
        } else if (gamepad1.b){
            gate.setPosition(0.6);
        } else if (gamepad1.y){
            gate.setPosition(0.85);
        } else if (gamepad1.x){
            gate.setPosition(0.0);
        }
        telemetry.addData("servo position", gate.getPosition());
    }

}
