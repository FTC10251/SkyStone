package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "HandyChin")
@Disabled()
public class HandyChin  extends OpMode {
    Servo leftIntake;
    Servo rightIntake;
    Servo bouncer;
    double bouncerPos = 0;

    public void init(){
        leftIntake = hardwareMap.get(Servo.class,"leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        bouncer = hardwareMap.get(Servo.class, "bouncer");
        rightIntake.setDirection(Servo.Direction.REVERSE);
        leftIntake.setPosition(0.5);
        rightIntake.setPosition(0.5);
        bouncer.setPosition(0);

    }
    public void loop(){
        if(gamepad1.left_bumper){
            leftIntake.setPosition(.05);
        }
        else{
            leftIntake.setPosition(0.5 + (gamepad1.left_trigger/2.3));
        }
        if(gamepad1.right_bumper){
            rightIntake.setPosition(.05);
        }
        else{
            rightIntake.setPosition(0.5 + (gamepad1.right_trigger/2.3));

        }
        if(gamepad1.dpad_up){
            bouncerPos = .58;
        }
        else if(gamepad1.dpad_down){
            bouncerPos = .73;
        }


        bouncer.setPosition(bouncerPos);
        telemetry.addData("Bouncer Position:", bouncer.getPosition());
        telemetry.update();


    }

}
