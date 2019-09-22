package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Arm Testing", group = "HDrive")
@Disabled()
public class ArmTesting extends OpMode {
    boolean wasXPressed = false;
    boolean horizontal = false;
    boolean elbowMoved = false;
    boolean shoulderMoved = false;
    boolean rotationMoved = false;

    int rotationPosition = 0;
    int shoulderPosition = 0;
    int elbowPosition = 0;

    double x = 10;
    double y = 0;

    double bicep = 18;
    double forearm = 17.25;

    long lastTime;
    long thisTime;

    DcMotorEx shoulderMotor;
    DcMotorEx elbowMotor;
    DcMotorEx rotationMotor;

    ArmCalculator2 calc;

    public void init() {
        lastTime = System.currentTimeMillis();

        shoulderMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Shoulder Motor");
        elbowMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Elbow Motor");
        rotationMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Rotation Motor");
        calc = new ArmCalculator2(18, 17.25);

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        boolean x = gamepad1.x;
        if (x == true) {
            if (!wasXPressed) {
                horizontal = !horizontal;
            }

            wasXPressed = x;
        }
        moveArmPosition();
    }

    public void moveArmPosition() {
        thisTime = System.currentTimeMillis();
        if (horizontal) {
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double xChange = gamepad1.left_stick_x * .005 * (thisTime - lastTime);
            double yChange = gamepad1.left_stick_y * .0025 * (thisTime - lastTime);
            x = x + xChange;
            y = y + yChange;
            double length = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            double shoulderAngle = Math.toDegrees(Math.acos(x / length) + Math.acos((Math.pow(forearm, 2) - Math.pow(bicep, 2) - Math.pow(length, 2)) / (2 * length * bicep)));
            double elbowAngle = Math.toDegrees(Math.acos((Math.pow(length, 2) - Math.pow(bicep, 2) - Math.pow(forearm, 2)) / (2 * bicep * forearm))) - 90 + shoulderAngle;
            int elbowPos = (int) ((elbowAngle - 90) * 2350 * 4 / 360);
            int shoulderPos = (int) ((152 - shoulderAngle) * (4318 - 1758) / 90);
            elbowMotor.setTargetPosition(elbowPos);
            shoulderMotor.setTargetPosition(shoulderPos);
            rotationMotor.setPower(gamepad1.right_stick_x * .25);
        } else {
            elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbowMotor.setPower(gamepad1.left_stick_y * .4);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderMotor.setPower(gamepad1.left_stick_x * .4);
            rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotationMotor.setPower(gamepad1.right_stick_x * .25);

        }
        lastTime = thisTime;
    }

    public void moveArmSpeed() {
        double rotationSpeed;
        double elbowSpeed;
        double shoulderSpeed;
        calc.calculateSpeed(gamepad1.left_stick_x, -gamepad1.right_stick_y, shoulderMotor, elbowMotor, ExcessStuff.shoulderAngle((double) shoulderMotor.getCurrentPosition()), ExcessStuff.elbowAngle((double) elbowMotor.getCurrentPosition()), telemetry);

        if (horizontal) {
            rotationSpeed = gamepad1.right_stick_x;
            elbowSpeed = .4 * calc.getElbowMotorSpeed();
            shoulderSpeed = .4 * calc.getShoulderMotorSpeed();
        } else {
            rotationSpeed = .5 * gamepad1.right_stick_x;
            elbowSpeed = .5 * -gamepad1.left_stick_y;
            shoulderSpeed = .5 * gamepad1.left_stick_x;
        }
        if (rotationSpeed != 0) {
            rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotationMotor.setPower(rotationSpeed);
            rotationMoved = true;
            rotationPosition = rotationMotor.getCurrentPosition();
        } else {
            rotationMoved = false;
            rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationMotor.setTargetPosition(rotationPosition);
        }
        if (elbowSpeed != 0) {
            elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbowMotor.setPower(elbowSpeed);
            elbowMoved = true;
            elbowPosition = elbowMotor.getCurrentPosition();
        } else {
            elbowMoved = false;
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setTargetPosition(elbowPosition);
        }
        if (shoulderSpeed != 0) {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderMotor.setPower(shoulderSpeed);
            shoulderMoved = true;
            shoulderPosition = shoulderMotor.getCurrentPosition();
        } else {
            shoulderMoved = false;
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(shoulderPosition);
        }

    }
}