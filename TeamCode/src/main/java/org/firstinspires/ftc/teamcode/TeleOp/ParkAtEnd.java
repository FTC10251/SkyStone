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
@Autonomous(name = "Park", group = "HDrive")
public class ParkAtEnd extends LinearOpMode {
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

        }
        ThreadSleepUpdated(25000);
        encoderDriveProfiled(.1,.1,.4,10,2,2,0,true);



        //Block is in Right Starting Position


        // Disable Tracking when we are done;
        //targetsSkyStone.deactivate();

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
            while (Math.abs(leftMotor.getCurrentPosition()) < Math.abs(newSideTargets) - 5 || Math.abs(rightMotor.getCurrentPosition()) < Math.abs(newSideTargets) - 5 && opModeIsActive()) {
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


