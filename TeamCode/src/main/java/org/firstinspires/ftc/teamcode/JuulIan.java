package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "JuulIan")
@Disabled()
public class JuulIan extends OpMode {
    Servo ian;

    public void init(){
        ian = hardwareMap.get(Servo.class,"ian");
        ian.setPosition(0.5);
    }

    public void loop(){

        ian.setPosition(0.5 + (gamepad1.left_stick_y/2.3));
    }
}
