package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Claw Test", group = "Claw")

public class ClawTester extends OpMode {

    BNO055IMU imu;
    boolean newYPressed2 = true;
    boolean newAPressed2 = true;
    int armPos = 0;

    Servo clawServo;

    public void init() {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setPosition(.5);
    }

    public void loop() {
        moveArm();
        telemetry.addData("test", 0);
        if (gamepad1.a) {
            clawServo.setPosition(.5);
        }
        if (gamepad1.b) {
            clawServo.setPosition(.28);
        }

        telemetry.update();

    }

    public void moveArm() {
        //closes claw and moves arm all the way over the robot/ back to home position
        if (gamepad2.y && newYPressed2) {
            newYPressed2 = false;
            switch (armPos) {
                case 0:
                    // currently home
                    clawServo.setPosition(.5);
                    break;
                case 1:
                    // currently extended
                    clawServo.setPosition(.28);
                    break;
                default:
                    telemetry.addData("Something went wrong and it is all darby's fault", 0);
            }

            armPos += 1;
            armPos = armPos % 2;
        } else if (!gamepad2.y) {
            newYPressed2 = true;

        }
        if (gamepad2.a && newAPressed2) {
            if (closeEnough(clawServo.getPosition(), /* open*/ .5, .05)) {
                clawServo.setPosition(/* closed*/.28);
            } else if (closeEnough(clawServo.getPosition(), /* closed*/ .28, .05)) {
                clawServo.setPosition(/* open*/.5);
            } else {
                telemetry.addData("Something went wrong and it is all darby's fault", 2);
            }
            newAPressed2 = false;
        } else if (!gamepad2.a) {
            newAPressed2 = true;
        }

    }


    public boolean closeEnough(double current, double target, double tolerance) {
        return Math.abs(current - target) <= tolerance;
    }

}
