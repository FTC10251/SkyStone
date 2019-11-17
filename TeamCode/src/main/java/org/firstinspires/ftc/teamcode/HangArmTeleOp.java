package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "HangArm")
public class HangArmTeleOp extends OpMode{
    int latchPos = 0;
    DcMotorEx hangMotor;
    Servo latch;
    boolean newXPressed = true;
    double armPos = .45;

    @Override
    public void init() {
        hangMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Hang Arm");
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        latch = hardwareMap.servo.get("Hang Arm Lock");
        latch.setPosition(.45);

    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper){
            hangMotor.setPower(.5);
        }
        else if (gamepad1.right_bumper){
            hangMotor.setPower(-.5);
        }
        else {
            hangMotor.setPower(0);
        }
        if(gamepad1.dpad_up) {
            armPos = armPos + .001;
            latch.setPosition(armPos);
        }
        else if(gamepad1.dpad_down) {
            armPos = armPos - .001;
            latch.setPosition(armPos);
        }
        else {
            armPos = armPos;
        }

        if (latchPos == 0 && gamepad1.x && newXPressed){
            latchPos = 1;
            newXPressed = false;
            telemetry.addData("Latch Position", "down");
            latch.setPosition(.55);
        }
        else if (latchPos == 1 && gamepad1.x && newXPressed){
            newXPressed = false;
            latchPos = 0;
            telemetry.addData("Latch Position", "up");
            latch.setPosition(.35);
        }
        else if (!gamepad1.x){
            newXPressed = true;
        }
        telemetry.addData("Latch Pos", armPos);
        telemetry.addData("Arm Pos", hangMotor.getCurrentPosition());
        telemetry.update();





    }
}
