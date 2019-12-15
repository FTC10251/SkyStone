package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;
import android.graphics.Point;
import android.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.ExtraClasses;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Created by Luke on 9/25/2016. Final Far Auto
 */
@Autonomous(name = "Competition Auto Red ", group = "HDrive")
public class CompAuto extends LinearOpMode {
    //Vuforia
    private static final String VUFORIA_KEY =
            "AQD7qV//////AAABmetA12LIIEPGqARlxRlpSeB78FHaKW23WxZ6eL692V0Xej0Jguf/s4qT3sxWQLGsv/gB/ewgs4P2Iqg08++3Uav5Tt/Pr27Jvr2sz6tvmPq3jENTD3l/kNUeno0Ko48xuc5xV8QT+7FkGuZ71BUQH4+iRXFWnQ7DBdQCL+flYnOOGxuNkfv0yVK9KhMlYifB/OAv+Ipkdex5orcWPnd2sXhKiHdLssleApDTBl+037zRwmiBZdJCvIJtf6bkk8qDB8+o2k0tlZ3IK2s8Kg9eNKCokA92wtTtsuqXGlNnrJOfT2kP85715fcmAwu6xzaq78q/7ay9zEiluf4bllNXNsg4CXNVIaIaWenkEFTUpyGB";
    static final double COUNTS_PER_INCH = /*(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*/ 45;

    BNO055IMU imu;
    ExtraClasses extraClasses;
    double initialAngle;
    double startPos;
    double angleDouble = 0;
    double blockPosition = 0;
    public static double holdServoPos = .58;
    public static double servoClosedPos = .15;
    DcMotorEx leftMotor;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor;
    DcMotorEx rightMotor2;
    DcMotorEx middleMotor;
    DcMotorEx leftIntakeMotor;
    DcMotorEx rightIntakeMotor;
    DcMotorEx arm;
    Servo rightIntakeServo;
    Servo leftIntakeServo;
    Servo hookServo;
    Servo clawServo;
    Servo holdServo;

    Orientation angles;
    PIDFCoefficients pidStuff;
    WebcamName webcamName = null;
    DistanceSensor rangeSensorLeft;
    DistanceSensor rangeSensorBack;
    TouchSensor touchSensor;

    //Vuforia Variables
    private static final float mmPerInch = 25.4f;
    private static final float stoneZ = 2.00f * mmPerInch;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters3 = new BNO055IMU.Parameters();
        parameters3.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters3.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters3.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters3.loggingEnabled = true;
        parameters3.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters3);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Left Motor Front");
        leftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "Left Motor Back");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Right Motor Front");
        rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "Right Motor Back");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Middle Motor");
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Intake Motor Right");
        leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Intake Motor Left");
        arm = (DcMotorEx) hardwareMap.get(DcMotor.class, "Arm");
        rightIntakeServo = hardwareMap.get(Servo.class, "Intake Servo Right");
        leftIntakeServo = hardwareMap.get(Servo.class, "Intake Servo Left");
        hookServo = hardwareMap.get(Servo.class, "Hook Servo");
        clawServo = hardwareMap.get(Servo.class, "Claw Servo");
        holdServo = hardwareMap.get(Servo.class, "Hold Servo");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        rangeSensorLeft = hardwareMap.get(DistanceSensor.class, "Range Sensor Left");
        rangeSensorBack = hardwareMap.get(DistanceSensor.class, "Range Sensor Back");
        touchSensor = hardwareMap.get(TouchSensor.class, "Touch Sensor");

        // set the digital channel to input.

        /*
         * Initialize the drive system variables.
         * The init
         * () method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders for awesome reason");    //
        telemetry.update();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(RUN_USING_ENCODER);
        leftMotor2.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        rightMotor2.setMode(RUN_USING_ENCODER);
        middleMotor.setMode(RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition(),
                middleMotor.getCurrentPosition());
        telemetry.update();
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 5;
        pidStuff.i = 1;
        pidStuff.d = 0;
        pidStuff.f = 10.2;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        leftMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        //pidStuff.f = 23;
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        initialAngle = angleDouble;

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addLine("Ready to Begin");
        telemetry.addData("Starting Angle", initialAngle);
        telemetry.update();
        hookServo.setPosition(.75);
        holdServo.setPosition(holdServoPos);
        waitForStart();
        Thread.sleep(20000);
        leftMotor.setPower(.05);
        leftMotor2.setPower(.05);
        rightMotor.setPower(.05);
        rightMotor2.setPower(.05);
        telemetry.addLine("Here");
        telemetry.update();
        Thread.sleep(2000);
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
        //Set Intake Position

        //Move forward to pick up block
        //move forward to pick up the founcation

    }

    public double convertAngle(double angle) {
        double angleUsed = angle + 180;
        return angleUsed;
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

    public void lignUpWithFoundation() {
        double distance = rangeSensorBack.getDistance(DistanceUnit.CM);
        double maxSpeed = -.5;
        double minSpeed = -.1;
        double goalDistance = 4;
        while (!touchSensor.isPressed()) {
            distance = rangeSensorBack.getDistance(DistanceUnit.CM);
            double sidePower = minSpeed + ((maxSpeed - minSpeed) * ((distance - goalDistance)) / 50);
            telemetry.addData("Distance", distance);
            telemetry.update();
            leftMotor.setPower(sidePower);
            leftMotor2.setPower(sidePower);
            rightMotor.setPower(sidePower);
            rightMotor2.setPower(sidePower);
        }
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        hookServo.setPosition(.3);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(STOP_AND_RESET_ENCODER);

        int newSideTargets = /*robot.leftMotor.getCurrentPosition() + */(int) ((Math.abs(-10)) * COUNTS_PER_INCH);
        leftMotor.setTargetPosition(newSideTargets);
        leftMotor2.setTargetPosition(newSideTargets);
        rightMotor.setTargetPosition(newSideTargets);
        rightMotor2.setTargetPosition(newSideTargets);

        leftMotor.setMode(RUN_TO_POSITION);
        leftMotor2.setMode(RUN_TO_POSITION);
        rightMotor.setMode(RUN_TO_POSITION);
        rightMotor2.setMode(RUN_TO_POSITION);
        while(leftMotor.isBusy() || leftMotor2.isBusy() && rightMotor.isBusy() && rightMotor2.isBusy()) {
            double currentPowerLeft = .3;
            double currentPowerRight = .3;

            double holdAngle = 180;
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
            double distance1 = Math.abs(angleDouble - holdAngle);
            double distance2 = Math.abs(Math.abs((360 - angleDouble)) - holdAngle);
            double angleError = distance1;
            if (distance1 > distance2) {
                angleError = distance2;
            }
            angleError = angleError / 30;
            if ((holdAngle - angleDouble + 360) % 360 < 180) {
                angleError = angleError;
            } else {
                angleError = angleError * -1;
            }
            leftMotor.setPower(currentPowerLeft - angleError);
            leftMotor2.setPower(currentPowerLeft - angleError);
            rightMotor.setPower(currentPowerRight + angleError);
            rightMotor2.setPower(currentPowerRight + angleError);
        }

        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
    }

    public void scoreFoundation() {
        leftMotor.setMode(RUN_USING_ENCODER);
        leftMotor2.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        rightMotor2.setMode(RUN_USING_ENCODER);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
        double scoreAngle = 90;
        double distance1 = Math.abs(angleDouble - scoreAngle);
        double distance2 = Math.abs(Math.abs((360 - angleDouble)) - scoreAngle);
        double angleError = distance1;
        if (distance1 > distance2) {
            angleError = distance2;
        }
        if ((scoreAngle - angleDouble + 360) % 360 < 180) {
            angleError = angleError;
        } else {
            angleError = angleError * -1;
        }
        double motorPower = angleError / 250;
        middleMotor.setPower(-.2);
        while (Math.abs(angleError) > 3) {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
            distance1 = Math.abs(angleDouble - scoreAngle);
            distance2 = Math.abs(Math.abs((360 - angleDouble)) - scoreAngle);
            angleError = distance1;
            if (distance1 > distance2) {
                angleError = distance2;
            }
            if ((scoreAngle - angleDouble + 360) % 360 < 180) {
                angleError = angleError;
            } else {
                angleError = angleError * -1;
            }
            motorPower = angleError / 250;
            double minSpeed = .05;
            leftMotor.setPower(-minSpeed - motorPower);
            leftMotor2.setPower(-minSpeed - motorPower);
            rightMotor.setPower(minSpeed + motorPower);
            rightMotor2.setPower(minSpeed + motorPower);
            telemetry.addData("Current Angle", angleDouble);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Angle Error", angleError);
            telemetry.update();
            if (Math.abs(angleError) < 50) {
                middleMotor.setPower(.2);
            }
        }
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
        middleMotor.setPower(0);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        double movePower = -.4;
        leftMotor.setPower(movePower);
        leftMotor2.setPower(movePower);
        rightMotor.setPower(movePower);
        rightMotor.setPower(movePower);
        long startingTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - startingTime) < 1000) {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
            scoreAngle = 90;
            distance1 = Math.abs(angleDouble - scoreAngle);
            distance2 = Math.abs(Math.abs((360 - angleDouble)) - scoreAngle);
            angleError = distance1;
            if (distance1 > distance2) {
                angleError = distance2;
            }
            if ((scoreAngle - angleDouble + 360) % 360 < 180) {
                angleError = angleError;
            } else {
                angleError = angleError * -1;
            }
            motorPower = angleError / 100;
            leftMotor.setPower(movePower - motorPower);
            leftMotor2.setPower(movePower - motorPower);
            rightMotor.setPower(movePower + motorPower);
            rightMotor2.setPower(movePower + motorPower);
        }
        hookServo.setPosition(.75);
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
    }
}


