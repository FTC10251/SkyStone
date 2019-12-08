package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.ArmCalculator;
import org.firstinspires.ftc.teamcode.ExcessStuff;
import org.firstinspires.ftc.teamcode.ExtraClasses;
import org.firstinspires.ftc.teamcode.HDriveFCCalc;
import org.firstinspires.ftc.teamcode.ManualImports.Point;

import java.util.ArrayList;
import java.util.Locale;


@TeleOp(name = "SkyStone TeleOp Updated For Bug", group = "HDrive")
@Disabled
public class TeleOpTestForBug extends LinearOpMode {
    HDriveFCCalc calculator;
    HDriveFCCalc dpadCalculator;
    ArmCalculator armCalculator;
    ExtraClasses extraClasses;
    FollowPath followPath;

    //Ints, Doubles, Booleans, and Floats
    int here = 0;
    double offset = 180;
    double bicep = 18;
    double forearm = 17.25;
    double speed = 1;
    double sideChangePower = 0;
    double leftX;
    double leftY;
    double holdAngle = 0;
    double timeDifferenceBetweenLoops = System.currentTimeMillis();
    double angleDouble = 0;
    double lastLeftPos = 0;
    double lastRightPos = 0;
    double lastMidPos = 0;
    double xPos = 0;
    double yPos = 0;
    double leftServoPos = .5;
    double rightServoPos = .5;
    double startingAngle = 0;
    double rangeValuePrior = 50;
    double rangeSensorDistanceMid = 50;
    double foundationState = 0;
    double startingTime = 0;
    double timeDifference = 0;
    double COUNTS_PER_INCH = 91.125;
    double autoScoreState = 0;
    double timeDifferencePosition = 0;
    double timeBefore = 0;
    double velX,velY,posX,posY;
    double autoScoringMode = 0;
    double blockPosY = 0;
    double blockPosX = 0;
    double rotation = 0;
    double previousX = 0;
    double previousY = 0;
    double previousTime = 0;

    boolean isPressedX = false;
    boolean armMode = false;
    boolean sideMoved = false;
    boolean dpadWasPressed = false;
    boolean updatedDpadMovement = false;
    boolean rightStickMoved = false;
    boolean leftTriggerState = false;
    boolean leftTriggerPressed = false;
    boolean aWasPressed = false;
    boolean driverHasControl = true;
    boolean autoScoringModeFirstTime = true;
    boolean filteredValues = false;
    boolean yWasPressed = false;
    boolean autoMovingFoundation = false;
    boolean pathFindingMode = false;
    boolean pathFindingModeFirstTime = true;
    boolean xWasPressed = false;
    boolean newAPressed2 = true;

    float rightX;
    long rightStickTimer = 0;
    Point velocity = new Point();





    //Robot Hardware
    DcMotorEx leftMotor;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor;
    DcMotorEx rightMotor2;
    DcMotorEx middleMotor;
    DcMotorEx rightIntakeMotor;
    DcMotorEx leftIntakeMotor;
    DcMotorEx armFlipper;
    Servo rightIntakeServo;
    Servo leftIntakeServo;
    Servo clawServo;
    Servo hookServo;
    WebcamName webcamName = null;

    DistanceSensor rangeSensorLeft;
    DistanceSensor rangeSensorBack;



    Orientation angles;
    BNO055IMU imu;
    PIDFCoefficients pidStuff;

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
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Left Motor Front");
        leftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "Left Motor Back");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Right Motor Front");
        rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "Right Motor Back");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Middle Motor");
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Intake Motor Right");
        leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Intake Motor Left");
        rightIntakeServo = hardwareMap.get(Servo.class, "Intake Servo Right");
        leftIntakeServo = hardwareMap.get(Servo.class, "Intake Servo Left");
        hookServo = hardwareMap.get(Servo.class, "Hook Servo");
        //clawServo = hardwareMap.get(Servo.class, "clawServo");
        //armFlipper = (DcMotorEx) hardwareMap.get(DcMotor.class, "armFlipper");
        rangeSensorBack = hardwareMap.get(DistanceSensor.class, "Range Sensor Back");
        rangeSensorLeft = hardwareMap.get(DistanceSensor.class, "Range Sensor Left");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        calculator = new HDriveFCCalc();
        dpadCalculator = new HDriveFCCalc();
        armCalculator = new ArmCalculator(bicep, forearm);
        extraClasses = new ExtraClasses(leftMotor, rightMotor, middleMotor, middleMotor);
        rightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);


        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        holdAngle = angleDouble;

        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 10;
        pidStuff.i = 10;
        pidStuff.d = 0;
        pidStuff.f = 14;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;

        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        leftMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);

        timeDifferenceBetweenLoops = System.currentTimeMillis();




        telemetry.addLine("Ready to Begin");
        telemetry.update();
        waitForStart();
        timeBefore = System.currentTimeMillis();
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            moveTheBase();
            telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
            telemetry.addData("Left Encoder 2", leftMotor2.getCurrentPosition());
            telemetry.addData("Right Encoder", rightMotor.getCurrentPosition());
            telemetry.addData("Right Encoder 2", rightMotor2.getCurrentPosition());
            telemetry.addData("Left mode", leftMotor.getMode());
            telemetry.addData("Right Mode", rightMotor.getMode());
            telemetry.addData("left 2 Mode", leftMotor2.getMode());
            telemetry.addData("right 2 Mode", rightMotor2.getMode());
            telemetry.update();
        }
    }





    public void moveTheBase() {
            leftMotor.setPower(gamepad1.left_stick_y);
            leftMotor2.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.left_stick_y);
            rightMotor2.setPower(gamepad1.left_stick_y);
    }

    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
