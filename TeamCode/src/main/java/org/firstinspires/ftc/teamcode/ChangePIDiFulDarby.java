package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Darby is petty")
public class ChangePIDiFulDarby extends OpMode {
    PIDFCoefficients pidStuff;
    double p;
    double i;
    double d;
    double f;
    int mode = 0;
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;
    boolean right_trigger_isPressed;
    boolean right_bumper_isPressed;
    boolean left_bumper_isPressed;
    double lastVal;
    boolean darby = false;

    @Override
    public void init() {
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor2");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor2.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 10;
        pidStuff.i = 2;
        pidStuff.d = 0;
        pidStuff.f = 14;
        p = 10;
        i = 2;
        d = 0;
        f = 14;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        right_trigger_isPressed = false;
        right_bumper_isPressed = false;
        left_bumper_isPressed = false;

    }

    @Override
    public void loop() {
        if (gamepad1. right_trigger > 0 && !right_trigger_isPressed){
            mode += 1;
            mode = mode%4;
            right_trigger_isPressed = true;
        }
        if (gamepad1.right_trigger == 0){
            right_trigger_isPressed = false;
        }
        if (mode == 0){
            p = changeValue(p, 1);
        }
        else if (mode == 1){
            i = changeValue(i, .25);
        }
        else if (mode == 2){
            d = changeValue(d, .25);
        }
        else if (mode == 3){
            f = changeValue(f, 1);
        }
        pidStuff.p = p;
        pidStuff.i = i;
        pidStuff.d = d;
        pidStuff.f = f;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        telemetry.addData("mode:", mode);
        telemetry.addData("P what I want to set", p);
        telemetry.addData( "P:", pidStuff.p);
        telemetry.addData( "I:", pidStuff.i);
        telemetry.addData( "D:", pidStuff.d);
        telemetry.addData( "F:", pidStuff.f);
        telemetry.addData("Left Pid F", leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addData("Did Change Val", darby);
        middleMotor.setPower(gamepad1.left_stick_x);
        middleMotor2.setPower(gamepad1.left_stick_x);

        if (gamepad1.right_stick_x != 0){
            rightMotor.setPower(gamepad1.right_stick_x);
            leftMotor.setPower(-gamepad1.right_stick_x);
        } else {
            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.left_stick_y);
        }
        if (Math.abs(leftMotor.getPower() + lastVal)< Math.abs (leftMotor.getPower())){
            telemetry.addData("braking", 1);
            darby = true;

        }
        if (darby && Math.abs(gamepad1.left_stick_x)< .05){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            darby = false;
        }

        lastVal = leftMotor.getPower();

        telemetry.update();
    }
    public double changeValue(double daniel, double incr){
        if (gamepad1.left_bumper && !left_bumper_isPressed){
            daniel -= incr;
            left_bumper_isPressed = true;
        }
        if (gamepad1.right_bumper && !right_bumper_isPressed){
            daniel += incr;
            right_bumper_isPressed = true;
        }
        if (!gamepad1.right_bumper){
            right_bumper_isPressed = false;
        }
        if (!gamepad1.left_bumper){
            left_bumper_isPressed = false;
        }
        return daniel;
    }
}
