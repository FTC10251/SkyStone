package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
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
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
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
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ExtraClasses;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
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
 *
 * Notes:
 * Positive Direction is intakeSide
 */
@Autonomous(name = "SkyStone Auto Updated (BLUE)", group = "HDrive")
public class SkyStoneAutonomousUpdatedBlue extends LinearOpMode {
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
    public static double servoOpenPos = .56;
    boolean firstTime = true;
    boolean finished = true;
    boolean filteredValues = false;
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
    Servo rotationServo;

    Orientation angles;
    PIDFCoefficients pidStuff;
    WebcamName webcamName = null;
    DistanceSensor rangeSensorRight;
    DistanceSensor rangeSensorBack;
    TouchSensor touchSensor;

    double reading1;
    double reading2;
    double reading3;
    double reading4;
    double redAverage = 0;
    double greenAverage = 0;
    double blueAverage = 0;

    //Vuforia Variables
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float stoneZ = 2.00f * mmPerInch;
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
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
        rotationServo = hardwareMap.get(Servo.class, "Rotation Servo");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        rangeSensorRight = hardwareMap.get(DistanceSensor.class, "Range Sensor Right");
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
        arm.setPIDFCoefficients(RUN_USING_ENCODER, pidStuff);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        initialAngle = angleDouble;

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        telemetry.addLine("Ready to Begin");
        telemetry.addData("Starting Angle", initialAngle);
        telemetry.update();
        targetsSkyStone.activate();
        //hookServo.setPosition(.75);
        while (!isStarted() && !isStopRequested()) {
            // check all the trackable targets to see which one (if any) is visible.
            vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
                @Override
                public void accept(Frame frame) {
                    Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                    if (bitmap != null) {
                        double bHeight = bitmap.getHeight();
                        double bWidth = bitmap.getWidth();
                        averagePixels(bWidth * (.375), bHeight * (.6666), bWidth * (.375), bHeight * (.75), bWidth * (.5), bHeight * (.666), bWidth * (.5), bHeight * (.75), bitmap);
                        double redAverage1 = redAverage;
                        double greenAverage1 = greenAverage;
                        double blueAverage1 = blueAverage;

                        averagePixels(bWidth * (.63), bHeight * (.666), bWidth * (.63), bHeight * (.75), bWidth * (.72), bHeight * (.66), bWidth * (.72), bHeight * (.75), bitmap);
                        double redAverage2 = redAverage;
                        double greenAverage2 = greenAverage;
                        double blueAverage2 = blueAverage;

                        averagePixels(bWidth * (.867), bHeight * (.6666), bWidth * (.867), bHeight * (.75), bWidth * (.933), bHeight * (.6666), bWidth * (.93), bHeight * (.75), bitmap);
                        double redAverage3 = redAverage;
                        double greenAverage3 = greenAverage;
                        double blueAverage3 = blueAverage;
                        if((redAverage1 + greenAverage1) / 2 < (redAverage2 + greenAverage2)/2 && (redAverage1 + greenAverage1)/2 < (redAverage3 + greenAverage3)/2) {
                            blockPosition = 2;
                        } else if((redAverage2 + greenAverage2) / 2 < (redAverage1 + greenAverage1)/2 && (redAverage2 + greenAverage2)/2 < (redAverage3 + greenAverage3)/2) {
                            blockPosition = 1;
                        } else if((redAverage3 + greenAverage3) / 2 < (redAverage1 + greenAverage1)/2 && (redAverage3 + greenAverage3)/2 < (redAverage2 + greenAverage2)/2) {
                            blockPosition = 0;
                        }

                        telemetry.addData("Block Position", blockPosition);
                        telemetry.addData("Red Average 1", redAverage1);
                        telemetry.addData("Green Average 1", greenAverage1);
                        telemetry.addData("Blue Average 1", blueAverage1);
                        telemetry.addData("Red Average 2", redAverage2);
                        telemetry.addData("Green Average 2", greenAverage2);
                        telemetry.addData("Blue Average 2", blueAverage2);
                        telemetry.addData("Red Average 3", redAverage3);
                        telemetry.addData("Green Average 3", greenAverage3);
                        telemetry.addData("Blue Average 3", blueAverage3);
                        telemetry.addData("Width", bitmap.getWidth());
                        telemetry.addData("Height", bitmap.getHeight());
                        telemetry.update();
                        int color = bitmap.getPixel(1, 1);
                        int red = Color.red(color);
                        int blue = Color.blue(color);
                        int green = Color.green(color);
                    }
                }
            }));

        }



        //Set Intake Position
        moveIntakeAndArm1();
        leftIntakeServo.setPosition(.96);
        rightIntakeServo.setPosition(.33);


        //Move forward to pick up block
        if(blockPosition == 0) { //Right
            encoderDriveProfiled(.2,.4,.5,11,1,6,0,false);
            turnInCircleProfiled(18,2,-1,35, .4,.1,.4,0,10,0);
            leftIntakeServo.setPosition(.55);
            rightIntakeServo.setPosition(.55);
            Thread.sleep(600);
            turnInCircleProfiled(15,2,-1,35, -.1,-.1,-.4,2,4,0);
            leftIntakeMotor.setPower(0);
            rightIntakeMotor.setPower(0);
            pickUpSkystone();

            //Turn towards foundation
            turnInPlace(-.05, 270, 3);
            ThreadSleepUpdated(400);
            encoderDriveProfiled(.1, .1, .5, 81, 2, 15, 270, true);
            ThreadSleepUpdated(100);
        } else if(blockPosition == 1) { //Mid
            encoderDriveProfiled(.2, .1, .5, 35, 1, 6, 0, true);
            leftIntakeServo.setPosition(.55);
            rightIntakeServo.setPosition(.55);
            ThreadSleepUpdated(500);

            //Move Back after picking up block
            encoderDriveProfiled(.2, .2, .5, -10, 1, 6, 0, true);

            ThreadSleepUpdated(100);
            leftIntakeMotor.setPower(0);
            rightIntakeMotor.setPower(0);
            pickUpSkystone();

            //Turn towards foundation
            turnInPlace(-.05, 270, 3);
            ThreadSleepUpdated(400);
            encoderDriveProfiled(.1, .1, .5, 81, 2, 15, 270, true);
            ThreadSleepUpdated(100);
        } else { //Right
                encoderDriveProfiled(.2,.4,.5,10,1,6,0,false);
            turnInCircleProfiled(20,2,1,25, .4,.1,.4,0,10,0);
            leftIntakeServo.setPosition(.55);
            rightIntakeServo.setPosition(.55);
            ThreadSleepUpdated(300);
            turnInCircleProfiled(10,2,1,25, -.1,-.1,-.4,2,4,0);

            ThreadSleepUpdated(100);
            leftIntakeMotor.setPower(0);
            rightIntakeMotor.setPower(0);
            pickUpSkystone();

            //Turn towards foundation
            turnInPlace(-.05, 270, 3);
            ThreadSleepUpdated(400);
            encoderDriveProfiled(.1, .1, .5, 81, 2, 15, 270, true);
        }



        //move towards foundation


        //turn towards foundation to hook it
        telemetry.addLine("Got here");
        telemetry.update();
        turnInPlace(-.05, 180, 3);
        ThreadSleepUpdated(100);

        autoScore();
        ThreadSleepUpdated(500);

        //move forward to pick up the founcation
        lignUpWithFoundation();
        ThreadSleepUpdated(200);

        //idk
        scoreFoundation();

        //score thing

        //finish under the foundation
        encoderDriveProfiled(.3, .2, .5, 41, 2, 5, 90, true);



        //Block is in Right Starting Position


        // Disable Tracking when we are done;
        //targetsSkyStone.deactivate();

    }

    public double convertAngle(double angle) {
        double angleUsed = angle + 180;
        return angleUsed;
    }

    public double offsetAngle(double angle) {
        double newAngle = angle;
        if (angle > 360) {
            newAngle = angle - 360;
        }
        return newAngle;
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

    public double roundDouble(double x) {
        DecimalFormat twoDForm = new DecimalFormat("0.########");
        String str = twoDForm.format(x);
        return Double.valueOf(str);
    }

    public double scaleSpeed(double maxSpeed, double minSpeed, double targetPos, double currentPos) {
        if (firstTime) {
            startPos = currentPos;
            firstTime = false;
            finished = false;
        }
        double speedDif = maxSpeed - minSpeed;
        double dis = targetPos - startPos;
        double midPt = startPos + dis / 2;
        if (Math.abs(currentPos) >= Math.abs(startPos) && Math.abs(currentPos) <= Math.abs(midPt)) {
            return minSpeed + ((currentPos - startPos) / (dis / 2)) * (speedDif);
        } else if (Math.abs(currentPos) <= Math.abs(targetPos) && Math.abs(currentPos) > Math.abs(midPt)) {
            return minSpeed + ((targetPos - currentPos) / (dis / 2)) * (speedDif);
        }
        if (Math.abs(currentPos) >= Math.abs(targetPos)) {
            firstTime = true;
            finished = true;
        }
        return 0;
    }

    public double scaleSpeed2(double maxSpeed, double minSpeed, double targetPos, double currentPos) {
        if (firstTime) {
            startPos = currentPos;
            firstTime = false;
            finished = false;
        }
        //  if(Math.abs(currentPos - startPos) <= Math.abs(targetPos - startPos)) {
        double scale = 1 - (Math.abs(currentPos - startPos)) / (Math.abs(targetPos - startPos));
        return scale * (maxSpeed - minSpeed) + minSpeed;
        //  }
        //   return 0;
    }

    public void turnInCircle(double radius, double velocity, double turnDirection, double rotations, double timeout) {
        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(STOP_AND_RESET_ENCODER);
        middleMotor.setMode(STOP_AND_RESET_ENCODER);

        leftMotor.setMode(RUN_USING_ENCODER);
        leftMotor2.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        rightMotor2.setMode(RUN_USING_ENCODER);
        middleMotor.setMode(RUN_USING_ENCODER);

        boolean finishedMotion = false;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double startingAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
        double leftPower = 0;
        double rightPower = 0;
        double leftRadius = 0;
        double rightRadius = 0;
        if (turnDirection < 0) { //counterclockwize is negative
            leftRadius = radius - 8.25;
            rightRadius = radius + 8.25;
            leftPower = (2 * velocity) / (1 + (rightRadius / leftRadius));
            rightPower = (rightRadius / leftRadius) * ((2 * velocity) / (1 + (rightRadius / leftRadius)));

            if (rightPower > 1) {
                leftPower = leftPower * (1 / rightPower);
                rightPower = 1;
            }
        } else {
            leftRadius = radius + 8.25;
            rightRadius = radius - 8.25;
            rightPower = (2 * velocity) / (1 + (leftRadius / rightRadius));
            leftPower = (leftRadius / rightRadius) * ((2 * velocity) / (1 + (leftRadius / rightRadius)));
            if (leftPower > 1) {
                rightPower = rightPower * (1 / leftPower);
                leftPower = 1;
            }
        }

        while (opModeIsActive() && !finishedMotion) {


            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            double currentAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
            double angleDifference = Math.max(currentAngle, startingAngle) - Math.min(currentAngle, startingAngle);
            if (angleDifference >= rotations) {
                finishedMotion = true;
            }

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Angle Diff", angleDifference);
            telemetry.update();

            leftMotor.setPower(leftPower);
            leftMotor2.setPower(leftPower);
            rightMotor.setPower(rightPower);
            rightMotor2.setPower(rightPower);
        }
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
    }

    public void turnInCircleProfiled(double radius, double velocity, double turnDirection, double rotations, double minSpeedInitial, double minSpeedFinal, double maxSpeed, double speedUpAt, double slowDownAt, double nextSpeed) {
        leftMotor.setMode(RUN_USING_ENCODER);
        leftMotor2.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        rightMotor2.setMode(RUN_USING_ENCODER);
        middleMotor.setMode(RUN_USING_ENCODER);

        boolean finishedMotion = false;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double startingAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
        double leftPower = 0;
        double rightPower = 0;
        double leftRatio;
        double rightRatio;
        double leftRadius;
        double rightRadius;
        if (turnDirection < 0) { //counterclockwize is negative
            leftRadius = radius - 8.25;
            rightRadius = radius + 8.25;
            leftRatio = (2 * velocity) / (1 + (rightRadius / leftRadius));
            rightRatio = (rightRadius / leftRadius) * ((2 * velocity) / (1 + (rightRadius / leftRadius)));

            if (rightPower > 1) {
                leftRatio = leftRatio * (1 / rightRatio);
                rightRatio = 1;
            }
        } else {
            leftRadius = radius + 8.25;
            rightRadius = radius - 8.25;
            rightRatio = 1 * (2 * velocity) / (1 + (leftRadius / rightRadius));
            leftRatio = 1 * (leftRadius / rightRadius) * ((2 * velocity) / (1 + (leftRadius / rightRadius)));
            if (leftPower > 1) {
                rightRatio = rightRatio * (1 / leftRatio);
                leftRatio = 1;
            }
        }
        double speedDifferenceInitial = maxSpeed - minSpeedInitial;
        minSpeedInitial = minSpeedInitial / (Math.max(Math.abs(leftRatio), Math.abs(rightRatio)));
        speedDifferenceInitial = speedDifferenceInitial / (Math.max(Math.abs(leftRatio), Math.abs(rightRatio)));
        double speedDifferenceFinal = maxSpeed - minSpeedFinal;
        minSpeedFinal = minSpeedFinal / (Math.max(Math.abs(leftRatio), Math.abs(rightRatio)));
        speedDifferenceFinal = speedDifferenceFinal / (Math.max(Math.abs(leftRatio), Math.abs(rightRatio)));
        double angleDifference = 0;
        while (opModeIsActive() && !finishedMotion) {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            double currentAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
            angleDifference = extraClasses.angleDistance(currentAngle, startingAngle);
            if (angleDifference >= rotations) {
                finishedMotion = true;
                leftMotor.setPower(nextSpeed);
                leftMotor2.setPower(nextSpeed);
                rightMotor.setPower(nextSpeed);
                rightMotor2.setPower(nextSpeed);
            }
            int currentState = 0;
            if (angleDifference <= speedUpAt) {
                currentState = 1;
                leftPower = (minSpeedInitial * leftRatio) + (leftRatio * (angleDifference / speedUpAt) * speedDifferenceInitial);
                rightPower = (minSpeedInitial * rightRatio) + (rightRatio * (angleDifference / speedUpAt) * speedDifferenceInitial);
            } else if (angleDifference > speedUpAt && angleDifference < (rotations - slowDownAt)) {
                currentState = 2;
                leftPower = (leftRatio * minSpeedInitial) + (leftRatio * speedDifferenceInitial);
                rightPower = (rightRatio * minSpeedInitial) + (rightRatio * speedDifferenceInitial);
            } else if (angleDifference > (rotations - slowDownAt)) {
                currentState = 3;
                leftPower = (minSpeedFinal * leftRatio) + (leftRatio * ((rotations - angleDifference) / slowDownAt) * speedDifferenceFinal);
                rightPower = (minSpeedFinal * rightRatio) + (rightRatio * ((rotations - angleDifference) / slowDownAt) * speedDifferenceFinal);
            }
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Angle Diff", angleDifference);
            telemetry.addData("Current State", currentState);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Starting Angle", startingAngle);
            telemetry.addData("State", currentState);
            telemetry.update();
            if (!finishedMotion) {
                leftMotor.setPower(leftPower);
                leftMotor2.setPower(leftPower);
                rightMotor.setPower(rightPower);
                rightMotor2.setPower(rightPower);
            }
        }
        leftMotor.setPower(nextSpeed);
        leftMotor2.setPower(nextSpeed);
        rightMotor.setPower(nextSpeed);
        rightMotor2.setPower(nextSpeed);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double currentAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
        telemetry.addData("Angle", angleDifference);
        telemetry.addData("Angle Current", currentAngle);
        telemetry.addData("Starting Angle", startingAngle);
        telemetry.update();
    }

    public void encoderDriveProfiled(double minSpeedInitial, double minSpeedFinal, double maxSpeed, double inches, double speedUpAt, double slowDownAt, double holdAngle, boolean end) {
        int newSideTargets = 0;
        speedUpAt = speedUpAt * COUNTS_PER_INCH;
        slowDownAt = slowDownAt * COUNTS_PER_INCH;
        double speedDifferenceInitial = maxSpeed - minSpeedInitial;
        double speedDifferenceFinal = maxSpeed - minSpeedFinal;
        if (opModeIsActive()) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor2.setMode(STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor2.setMode(STOP_AND_RESET_ENCODER);

            newSideTargets = /*robot.leftMotor.getCurrentPosition() + */(int) ((inches) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newSideTargets);
            leftMotor2.setTargetPosition(newSideTargets);
            rightMotor.setTargetPosition(newSideTargets);
            rightMotor2.setTargetPosition(newSideTargets);

            leftMotor.setMode(RUN_USING_ENCODER);
            leftMotor2.setMode(RUN_USING_ENCODER);
            rightMotor.setMode(RUN_USING_ENCODER);
            rightMotor2.setMode(RUN_USING_ENCODER);

            double currentPowerLeft = 0;
            double currentPowerRight = 0;
            while (Math.abs(leftMotor.getCurrentPosition()) < Math.abs(newSideTargets)-5 || Math.abs(rightMotor.getCurrentPosition()) < Math.abs(newSideTargets) - 5 && opModeIsActive()) {
                int stateAt = 0;
                double currentEncoderPos = leftMotor.getCurrentPosition();
                if (Math.abs(currentEncoderPos) < Math.abs(speedUpAt)) {
                    stateAt = 1;
                    currentPowerLeft = minSpeedInitial + (speedDifferenceInitial * (Math.abs(currentEncoderPos / speedUpAt)));
                    currentPowerRight = minSpeedInitial + (speedDifferenceInitial * (Math.abs(currentEncoderPos / speedUpAt)));
                } else if (Math.abs(currentEncoderPos) >= Math.abs(speedUpAt) && Math.abs(currentEncoderPos) <= (newSideTargets - slowDownAt)) {
                    stateAt = 2;
                    currentPowerLeft = minSpeedInitial + speedDifferenceInitial;
                    currentPowerRight = minSpeedInitial + speedDifferenceInitial;
                } else if (Math.abs(currentEncoderPos) > (newSideTargets - slowDownAt)) {
                    stateAt = 3;
                    currentPowerLeft = minSpeedFinal + (speedDifferenceFinal * ((newSideTargets - Math.abs(currentEncoderPos)) / slowDownAt));
                    currentPowerRight = minSpeedFinal + (speedDifferenceFinal * ((newSideTargets - Math.abs(currentEncoderPos)) / slowDownAt));
                }

                angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                angleDouble = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
                double distance1 = Math.abs(angleDouble - holdAngle);
                double distance2 = Math.abs(Math.abs((360 - angleDouble)) - holdAngle);
                double angleError = distance1;
                if (distance1 > distance2) {
                    angleError = distance2;
                }
                angleError = angleError / 60;
                if ((holdAngle - angleDouble + 360) % 360 < 180) {
                    angleError = angleError;
                } else {
                    angleError = angleError * -1;
                }
                leftMotor.setPower(currentPowerLeft - angleError);
                leftMotor2.setPower(currentPowerLeft - angleError);
                rightMotor.setPower(currentPowerRight + angleError);
                rightMotor2.setPower(currentPowerRight + angleError);

                telemetry.addData("Power", currentPowerLeft);
                telemetry.addData("At", stateAt);
                telemetry.addData("Target", newSideTargets);
                telemetry.addData("Current Pos", leftMotor.getCurrentPosition());
                telemetry.addData("Current Right", rightMotor.getCurrentPosition());
                telemetry.addData("Angle Error", angleError);
                telemetry.addData("Angle Current", extraClasses.convertAngle(angleDouble));
                telemetry.update();
            }
        }
        if (end == true) {
            leftMotor.setPower(0);
            leftMotor2.setPower(0);
            rightMotor.setPower(0);
            rightMotor2.setPower(0);
        } else {
            leftMotor.setPower(minSpeedFinal);
            leftMotor2.setPower(minSpeedFinal);
            rightMotor.setPower(minSpeedFinal);
            rightMotor2.setPower(minSpeedFinal);
        }
    }



    public void turnInPlace(double minSpeed, double turnAngle, double errorAllowed) {
        leftMotor.setMode(RUN_USING_ENCODER);
        leftMotor2.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        rightMotor2.setMode(RUN_USING_ENCODER);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        double distance1 = Math.abs(angleDouble - turnAngle);
        double distance2 = Math.abs(Math.abs((360 - angleDouble)) - turnAngle);
        double angleError = distance1;
        if (distance1 > distance2) {
            angleError = distance2;
        }
        while (Math.abs(angleError) > errorAllowed) {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
            distance1 = Math.abs(angleDouble - turnAngle);
            distance2 = Math.abs(Math.abs((360 - angleDouble)) - turnAngle);
            angleError = distance1;
            if (distance1 > distance2) {
                angleError = distance2;
            }
            if ((turnAngle - angleDouble + 360) % 360 < 180) {
                angleError = angleError;
            } else {
                angleError = angleError * -1;
            }
            angleError = angleError / 250;
            telemetry.addData("Current Angle", angleDouble);
            telemetry.addData("Goal Angle", turnAngle);
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.addData("Angle Error", angleError * 250);
            telemetry.addData("angle error", angleError);
            telemetry.update();
            leftMotor.setPower(-minSpeed - angleError);
            leftMotor2.setPower(-minSpeed - angleError);
            rightMotor.setPower(minSpeed + angleError);
            rightMotor2.setPower(minSpeed + angleError);
            angleError = angleError * 250;
        }
        telemetry.addData("Angle Error", angleError);
        telemetry.addLine("Done");
        telemetry.update();
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
    }

    public void moveAwayFromWall() {
        double distance = rangeSensorRight.getDistance(DistanceUnit.CM);
        while (distance < 100) {
            distance = rangeSensorRight.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
    }

    public void lignUpWithFoundation() {
        double distance = rangeSensorBack.getDistance(DistanceUnit.CM);
        double maxSpeed = -.4;
        double minSpeed = -.1;
        double goalDistance = 4;
        while (!touchSensor.isPressed()) {
            distance = rangeSensorBack.getDistance(DistanceUnit.CM);
            if(distance > 30) {
                distance = 30;
            }
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
        ThreadSleepUpdated(300);

        hookServo.setPosition(.3);
        ThreadSleepUpdated(1500);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(STOP_AND_RESET_ENCODER);

        int newSideTargets = /*robot.leftMotor.getCurrentPosition() + */(int) ((Math.abs(-16)) * COUNTS_PER_INCH);
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
        middleMotor.setPower(.2);
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
            double minSpeed = -.05;
            leftMotor.setPower(-minSpeed - motorPower);
            leftMotor2.setPower(-minSpeed - motorPower);
            rightMotor.setPower(minSpeed + motorPower);
            rightMotor2.setPower(minSpeed + motorPower);
            telemetry.addData("Current Angle", angleDouble);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Angle Error", angleError);
            telemetry.update();
            if (Math.abs(angleError) < 50) {
                middleMotor.setPower(-.2);
            }
        }
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
        middleMotor.setPower(0);
        ThreadSleepUpdated(500);
        double movePower = -.4;
        leftMotor.setPower(movePower);
        leftMotor2.setPower(movePower);
        rightMotor.setPower(movePower);
        rightMotor.setPower(movePower);
        long startingTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - startingTime) < 1300) {
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

    public void autoScore() {
        int goalArmPos = -3500;
        double goalAngle = 180;
        double setDistanceItShouldBeBack = 16;
        double setDistanceItShouldBeMid = 51;
        double distanceDifferenceMid = 100;
        double distanceDifferenceBack = rangeSensorBack.getDistance(DistanceUnit.CM) - setDistanceItShouldBeBack;
        arm.setTargetPosition(goalArmPos);
        arm.setMode(RUN_TO_POSITION);
        arm.setPower(.5);
        double startingPos = arm.getCurrentPosition();
        boolean scored = false;
        boolean finishedMovementOverRobot = false;
        int armState = 0;
        double previousValue = rangeSensorBack.getDistance(DistanceUnit.CM);
        int readingNum = 1;
        while(!scored) {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));

            goalAngle = 180; //Just the starting angle I think
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
            double rangeSensorDistanceBack = rangeSensorBack.getDistance(DistanceUnit.CM);
            double filteredRangeSensorDistanceBack = filterValues(rangeSensorDistanceBack, readingNum);
            distanceDifferenceBack = filteredRangeSensorDistanceBack - setDistanceItShouldBeBack;
            double frontPowerError = distanceDifferenceBack / 75;

            //Find how far from the foundation it is
            double rangeSensorDistanceMid = rangeSensorRight.getDistance(DistanceUnit.CM);
            double rangeSensorValueUsed = rangeSensorDistanceMid;
            distanceDifferenceMid = rangeSensorValueUsed - setDistanceItShouldBeMid;
            double middlePowerError = distanceDifferenceMid / 35;
            if(frontPowerError > .2) {
                frontPowerError = .2;
            } else if(frontPowerError < -.2) {
                frontPowerError = -.2;
            }

            leftMotor.setPower(-frontPowerError + angleAdjustPower);
            leftMotor2.setPower(-frontPowerError + angleAdjustPower);
            rightMotor.setPower(-frontPowerError + -angleAdjustPower);
            rightMotor2.setPower(-frontPowerError + -angleAdjustPower);
            middleMotor.setPower(-middlePowerError);
            telemetry.addData("Distance", rangeSensorBack.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance Filtered", rangeSensorValueUsed);
            telemetry.addData("Angle", angleDouble);
            telemetry.addData("Angle Difference", angleError);
            telemetry.addData("Mid Reading", rangeSensorDistanceMid);
            telemetry.addData("Back Reading", rangeSensorDistanceBack);
            telemetry.addData("Mid Difference", distanceDifferenceMid);
            telemetry.addData("Back Difference", distanceDifferenceBack);
            telemetry.addData("Front Power Error", frontPowerError);
            telemetry.addData("angle Adjust Power", angleAdjustPower);
            telemetry.addData("Arm", arm.getCurrentPosition());
            telemetry.update();

            if(!extraClasses.closeEnough(arm.getCurrentPosition(), goalArmPos, 30) && opModeIsActive() && !finishedMovementOverRobot) {
                double speed = .1 + (.5 * (1 - ((arm.getCurrentPosition()) - startingPos) / goalArmPos));
                telemetry.addData("Speed", speed);
                telemetry.update();
                arm.setPower(speed);
                finishedMovementOverRobot = true;
            }
            else if(Math.abs(distanceDifferenceBack) < 2 && Math.abs(distanceDifferenceMid) < 2 && arm.getCurrentPosition() < goalArmPos + 100) {
                if(armState == 0) {
                    arm.setTargetPosition(-3600);
                    arm.setPower(.3);
                    if(extraClasses.closeEnough(arm.getCurrentPosition(),-3600,25)) {
                        arm.setPower(0);
                        clawServo.setPosition(servoOpenPos);
                        ThreadSleepUpdated(1000);
                        armState = 1;
                    }
                } else if(armState == 1) {
                    arm.setTargetPosition(0);
                    arm.setPower(.5);
                    scored = true;
                }
            }
            readingNum++;
        }
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
        middleMotor.setPower(0);
        telemetry.addLine("Done");
        telemetry.update();
    }

    public void moveIntakeAndArm1() {
        holdServo.setPosition(holdServoPos);
        rotationServo.setPosition(.54);
        arm.setTargetPosition(-500);
        arm.setMode(RUN_TO_POSITION);
        arm.setPower(.5);
        clawServo.setPosition(servoOpenPos);
        if(blockPosition == 0) {
            leftIntakeServo.setPosition(.3);
            rightIntakeServo.setPosition(.1);
        } else if(blockPosition == 1) {
            leftIntakeServo.setPosition(0);
            rightIntakeServo.setPosition(.1);
        } else {
            leftIntakeServo.setPosition(0);
            rightIntakeServo.setPosition(.1);
        }
        leftIntakeMotor.setPower(1);
        rightIntakeMotor.setPower(1);
    }
    public void moveIntakeAndArm2() {
        holdServo.setPosition(holdServoPos);
        rotationServo.setPosition(.54);
        arm.setTargetPosition(-500);
        arm.setMode(RUN_TO_POSITION);
        arm.setPower(.5);
        clawServo.setPosition(servoOpenPos);
        leftIntakeServo.setPosition(.6);
        rightIntakeServo.setPosition(0);
        leftIntakeMotor.setPower(1);
        rightIntakeMotor.setPower(1);
    }
    public void pickUpSkystone() {
        leftIntakeServo.setPosition(1);
        rightIntakeServo.setPosition(.4);
        holdServo.setPosition(holdServoPos);
        rotationServo.setPosition(.54);
        arm.setTargetPosition(-0);
        arm.setTargetPositionTolerance(1);
        arm.setMode(RUN_TO_POSITION);
        arm.setPower(.5);

        while(!extraClasses.closeEnough(arm.getCurrentPosition(), -50,5)) {
            arm.setPower(.5);
            telemetry.addData("Arm Pos",arm.getCurrentPosition());
            telemetry.addData("Get PIDF Coef", arm.getPIDFCoefficients(RUN_USING_ENCODER));
            telemetry.update();
        }
        ThreadSleepUpdated(100);
        clawServo.setPosition(servoClosedPos);
        ThreadSleepUpdated(300);
        arm.setTargetPosition(-210);
        arm.setPower(.4);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public int getPosition(double xTranslation) {
        int position = 4; //0 is left, 1 is middle, 2 is right
        if (xTranslation < -15) {
            position = 0;
        } else if (xTranslation > -15 && xTranslation < 15) {
            position = 1;
        } else if (xTranslation > 15) {
            position = 2;
        }
        return position;
    }

    public void ThreadSleepUpdated(double waitTime) {
        double startingTime = System.currentTimeMillis();
        boolean isDone = false;
        while(!isDone && opModeIsActive()) {
            double timeDifference = System.currentTimeMillis() - startingTime;
            if(timeDifference > waitTime) {
                isDone = true;
            }
        }
    }
    /*public double filterValues(double currentValue, double previousValue, double tolerance) {
        if(Math.abs(currentValue - previousValue) < tolerance) {
            filteredValues = false;
            return currentValue;
        } else {
            filteredValues = true;
            return previousValue;
        }
    }*/

    public double filterValues(double sensorReading, int readingNum){
        sensorReading = rangeSensorBack.getDistance(DistanceUnit.CM);
        if (readingNum % 4 == 1){
            reading1 = sensorReading;
            if(readingNum == 1){
                if(reading1 > 60) {
                    return 60;
                } else {
                    return reading1;
                }
            }
        }
        else if (readingNum % 4 == 2){
            reading2 = sensorReading;
            if(readingNum == 2){
                if(((reading1 + reading2) / 2) > 60) {
                    return 60;
                } else {
                    return (reading1 + reading2) / 2;
                }
            }
        }
        else if (readingNum % 4 == 3){
            reading3 = sensorReading;
            if(readingNum == 3){
                if((reading1 + reading2 + reading3)/3 > 60) {
                    return 60;
                } else {
                    return (reading1 + reading2 + reading3) / 3;
                }
            }
        }
        else if (readingNum % 4 == 0){
            reading4 = sensorReading;
        }
        //if (readingNum >= 4){
        if(byeByeValue(reading1,reading2,reading3,reading4) > 60) {
            return 60;
        }
        return byeByeValue(reading1, reading2, reading3, reading4);
        //}
        //return 0;
    }
    public double byeByeValue (double val1, double val2, double val3, double val4){
        double[] values = {val1,val2,val3,val4};
        Arrays.sort(values);
        return (values[1]+values[2])/2;
    }
    public void averagePixels(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, Bitmap bitmap) {
        int color1 = bitmap.getPixel((int)x1,(int)y1);
        int color2 = bitmap.getPixel((int)x2,(int)y2);
        int color3 = bitmap.getPixel((int)x3,(int)y3);
        int color4 = bitmap.getPixel((int)x4,(int)y4);

        redAverage = ((Color.red(color1) + Color.red(color2) + Color.red(color3) + Color.red(color4)))/4;
        blueAverage = ((Color.blue(color1) + Color.blue(color2) + Color.blue(color3) + Color.blue(color4)))/4;
        greenAverage = ((Color.green(color1) + Color.green(color2) + Color.green(color3) + Color.green(color4)))/4;
    }


}


