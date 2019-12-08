package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name = "Hirsh's stupid program")
public class RoboPosTune extends LinearOpMode {
    BNO055IMU imu;
    ExtraClasses extraClasses;
    Orientation angles;
    DistanceSensor rangeSensorLeft;
    DistanceSensor rangeSensorBack;
    DcMotorEx leftMotor;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor;
    DcMotorEx rightMotor2;
    DcMotorEx middleMotor;
    DcMotorEx leftIntakeMotor;
    DcMotorEx rightIntakeMotor;
    DcMotorEx arm;
    Servo clawServo;
    double angleDouble = 0;
    double mid = 58;
    double back = 24;
    private boolean newLeftTriggerPressed = true;
    private boolean newRightTriggerPressed = true;
    private boolean newLeftBumperPressed = true;
    private boolean newRightBumperPressed = true;



    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Left Motor Front");
        leftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "Left Motor Back");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Right Motor Front");
        rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "Right Motor Back");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Middle Motor");
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Intake Motor Right");
        leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Intake Motor Left");
        arm = (DcMotorEx) hardwareMap.get(DcMotor.class, "Arm");
        clawServo = hardwareMap.get(Servo.class, "Claw Servo");
        rangeSensorLeft = hardwareMap.get(DistanceSensor.class, "Range Sensor Left");
        rangeSensorBack = hardwareMap.get(DistanceSensor.class, "Range Sensor Back");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        clawServo.setPosition(.15);
        waitForStart();
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));

            if (gamepad1.left_trigger != 0 && newLeftTriggerPressed){
                back++;
                newLeftTriggerPressed = false;

            }else if (gamepad1.right_trigger !=0  && newRightTriggerPressed){
                mid++;
                newRightTriggerPressed = false;
            }
            if (gamepad1.left_bumper && newLeftBumperPressed){
                back--;
                newLeftBumperPressed = false;

            }else if (gamepad1.right_bumper && newRightBumperPressed){
                mid--;
                newRightBumperPressed = false;
            }
            if (gamepad1.left_trigger == 0){
                newLeftTriggerPressed = true;
            }
            if (gamepad1.right_trigger == 0){
                newRightTriggerPressed = true;
            }
            if (!gamepad1.left_bumper){
                newLeftBumperPressed = true;
            }
            if (!gamepad1.right_bumper){
                newRightBumperPressed = true;
            }
            autoScore(mid, back);

        }
    }
    public void autoScore(double mid, double back) {
        double goalAngle = 0;
        double distanceDifferenceMid = 100;
        double distanceDifferenceBack = 100;
        if(!extraClasses.closeEnough(angleDouble, goalAngle, 3) || Math.abs(distanceDifferenceMid) > 2 || Math.abs(distanceDifferenceBack) > 2) {

            goalAngle = 0; //Just the starting angle I think
            double distance1 = Math.abs(angleDouble - goalAngle);
            double distance2 = Math.abs(Math.abs((360 - angleDouble)) - goalAngle);
            double angleError = distance1;
            double angleAdjustPower = 0;
            if (distance1 > distance2) {
                angleError = distance2;
            }
            if ((goalAngle - angleDouble + 360) % 360 < 180) {
                angleError = angleError * -1;
            } else {
                angleError = angleError;
            }
            angleAdjustPower = angleError / 60;

            //Find how far from the side wall it is
            double setDistanceItShoudldBeBack = back;
            double rangeSensorDistanceBack = rangeSensorBack.getDistance(DistanceUnit.CM);
            distanceDifferenceBack = rangeSensorDistanceBack - setDistanceItShoudldBeBack;
            double frontPowerError = distanceDifferenceBack / 35;

            //Find how far from the foundation it is
            double setDistanceItShoudldBeMid = mid;
            double rangeSensorDistanceMid = rangeSensorLeft.getDistance(DistanceUnit.CM);
            double rangeSensorValueUsed = rangeSensorDistanceMid;
            distanceDifferenceMid = rangeSensorValueUsed - setDistanceItShoudldBeMid;
            double middlePowerError = distanceDifferenceMid / 15;

            leftMotor.setPower(-frontPowerError + angleAdjustPower);
            leftMotor2.setPower(-frontPowerError + angleAdjustPower);
            rightMotor.setPower(-frontPowerError + -angleAdjustPower);
            rightMotor2.setPower(-frontPowerError + -angleAdjustPower);
            middleMotor.setPower(middlePowerError);
            telemetry.addData("Angle", angleDouble);
            telemetry.addData("Angle Difference", angleError);
            telemetry.addData("Mid Reading", rangeSensorDistanceMid);
            telemetry.addData("Back Reading", rangeSensorDistanceBack);
            telemetry.addData("Mid Difference", distanceDifferenceMid);
            telemetry.addData("Back Difference", distanceDifferenceBack);
            telemetry.addData("Back Distance", rangeSensorBack.getDistance(DistanceUnit.CM));
            telemetry.addData("Mid Distance", rangeSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        else {
            leftMotor.setPower(0);
            leftMotor2.setPower(0);
            rightMotor.setPower(0);
            rightMotor2.setPower(0);
            middleMotor.setPower(0);
        }

    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
