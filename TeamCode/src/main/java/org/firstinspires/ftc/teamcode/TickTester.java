package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "Tick Tester")
public class TickTester extends OpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1150 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = /*(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*/45;
    static final double COUNTS_PER_INCH_SIDE = 125;
    DcMotorEx leftMotor;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor;
    DcMotorEx rightMotor2;
    DcMotorEx middleMotor;
    DcMotorEx arm;

    Servo leftIntakeServo;
    Servo rightIntakeServo;
    Servo hookServo;
    TouchSensor touchSensor;

    String angleDouble = "hi";
    Orientation angles;
    BNO055IMU imu;
    PIDFCoefficients pidStuff;
    DistanceSensor rangeSensor;
    DistanceSensor rangeSensorLeft;
    boolean state = false;
    boolean isPressedX = false;
    boolean isPressedTrigger = false;
    boolean isPressedBumper = false;
    boolean runMotors = false;
    int counter = 150;
    int leftEncoders = 0;
    int rightEncoders = 0;
    int middleEncoders = 0;
    int changed = 0;

    int leftEncodersFinal = 0;
    int rightEncodersFinal = 0;
    int middleEncodersFinal = 0;

    int leftChange = 0;
    int rightChange = 0;
    int middleChange = 0;

    double leftChangeInches = 0;
    double rightChangeInches = 0;
    double middleChangeInches = 0;
    double bouncerPos = 0;
    double leftIntakeServoPos = .5;
    double rightIntakeServoPos = 1;

    @Override
    public void init() {
        /** Wait for the game to begin */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        leftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"Left Motor Front");
        leftMotor2 = (DcMotorEx)hardwareMap.get(DcMotor.class, "Left Motor Back");
        rightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "Right Motor Front");
        rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "Right Motor Back");
        middleMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "Middle Motor");
        arm = (DcMotorEx) hardwareMap.get(DcMotor.class, "Arm");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "Range Sensor Back");
        rangeSensorLeft = hardwareMap.get(DistanceSensor.class, "Range Sensor Left");
        rightIntakeServo = hardwareMap.get(Servo.class, "Intake Servo Right");
        leftIntakeServo = hardwareMap.get(Servo.class, "Intake Servo Left");
        touchSensor = hardwareMap.get(TouchSensor.class, "Touch Sensor");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 5;
        pidStuff.i = 1;
        pidStuff.d = 0;
        pidStuff.f = 10.2;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        leftMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        rightMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);

    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper) {
            leftIntakeServoPos = leftIntakeServoPos + .02;
        } else if(gamepad1.left_trigger == 1) {
            leftIntakeServoPos = leftIntakeServoPos - .02;
        }
        if(gamepad1.right_bumper) {
            rightIntakeServoPos = rightIntakeServoPos + .02;
        } else if(gamepad1.right_trigger == 1) {
            rightIntakeServoPos = rightIntakeServoPos - .02;
        }
        leftIntakeServo.setPosition(leftIntakeServoPos);
        rightIntakeServo.setPosition(rightIntakeServoPos);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        telemetry.addData("Arm", arm.getCurrentPosition());
        telemetry.addData("Arm Angle", armAngle(arm.getCurrentPosition()));
        telemetry.addData("Hook Servo should", leftIntakeServoPos);
        telemetry.addData("Left Motor ", leftMotor.getCurrentPosition());
        telemetry.addData("Left Motor 2", leftMotor2.getCurrentPosition());
        telemetry.addData("Right Motor", rightMotor.getCurrentPosition());
        telemetry.addData("Right motor2 ", rightMotor2.getCurrentPosition());
        telemetry.addData("PRedicted Distance", leftMotor.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("Angle", ExtraClasses.convertAngle(Double.parseDouble(angleDouble)));
        telemetry.addData("Touch Sensor", touchSensor.isPressed());
        telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance", rangeSensorLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Intake", leftIntakeServo.getPosition());
        telemetry.addData("Right Intake", rightIntakeServo.getPosition());
        telemetry.update();
    }
    public double armAngle (double currentArmPos) {
        double angle = Math.abs(currentArmPos) / 13.333 + 50;
        return angle;
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public double convertAngle(String angle) {
        double angleUsed = Double.parseDouble(angle);
        if(angleUsed < 0) {
            angleUsed = 180 + (180-Math.abs(angleUsed));
        }
        else {
            angleUsed = angleUsed;
        }
        return angleUsed;
    }
}
