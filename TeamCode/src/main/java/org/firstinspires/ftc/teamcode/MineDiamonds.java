/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name= "MineDiamonds")
public class MineDiamonds extends OpMode{
    ArmCalculator calculator;
    DcMotor verticalMotor;
    DcMotor horizontalMotor;


    public void init(){
        verticalMotor = hardwareMap.get(DcMotor.class, "verticalMotor");
        horizontalMotor = hardwareMap.get(DcMotor.class, "horizontalMotor");
        calculator = new ArmCalculator(5, 5);
        verticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void loop(){
        float leftX = gamepad1.left_stick_x;
        float leftY = gamepad1.left_stick_y;
        calculator.calculateSpeed(leftX, leftY, verticalMotor, horizontalMotor);
        verticalMotor.setPower(calculator.getShoulderMotorSpeed());
        horizontalMotor.setPower(calculator.getElbowMotorSpeed());


    }
}
*/