package org.firstinspires.ftc.teamcode.Autonomous;
import android.graphics.Color;
import android.graphics.Point;
import android.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
@Autonomous(name= "SkyStone Auto", group = "HDrive")
public class SkyStoneAutonomous extends LinearOpMode {
    //Vuforia
    private static final String VUFORIA_KEY =
            "Aff5Onj/////AAABmWU00gJ3W031oMf/yab8x+lDTcSp4Ipj3tbjxGCvrixJIcImUGnhiWcbd+U0kjaqRapa8gT1pti8r6x9DZjPKc1Z33pt6uSXJ5nU6HliFApbRiW3nfGQjMaXbItmKyFyA4kkVbxz3Q/UHL5eQbOwpBR880DPsrqElLqI/A6d9MtxwR+HxPGwKjkGI/PQ8LLkoRDK0ea609oynQOlKybmI/a923orWWI+oPWmB6+41EbnAjtnxVK6BZ6e0Y8rXebNcD7/OjVsdZ+pTEfJzR4YYyDt7yCmLKknqW0WGdF0D0zBBBAt4sYwtB6e9CA8FxjxzDOMoUmlZ/pQFXwSdW4I07ZRzatuXJ2FG+hSvxATv1BG";
    static final double     COUNTS_PER_INCH         = /*(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*/ 91.125;

    BNO055IMU imu;
    ExtraClasses extraClasses;
    double initialAngle;
    double startPos;
    double angleDouble = 0;
    boolean firstTime = true;
    boolean finished = true;
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;

    Orientation angles;
    PIDFCoefficients pidStuff;
    WebcamName webcamName = null;

    //Vuforia Variables
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float stoneZ = 2.00f * mmPerInch;
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    @Override public void runOpMode() throws InterruptedException {

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
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor2");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Initialize the drive system variables.
         * The init
         * () method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders for awesome reason");    //
        telemetry.update();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition(),
                middleMotor.getCurrentPosition());
        telemetry.update();
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor2.setDirection(DcMotor.Direction.REVERSE);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 5;
        pidStuff.i = 1;
        pidStuff.d = 0;
        pidStuff.f = 10.2;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        //pidStuff.f = 23;
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        initialAngle = angleDouble;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        telemetry.addLine("Ready to Begin");
        telemetry.addData("Starting Angle", initialAngle);
        telemetry.update();

        waitForStart();

        encoderDriveProfiled(.1,.7,27,.1, 5,true);
        turnInCircleProfiled(10,.2,-1,90,-.1,-.7,.1,20);
        encoderDriveProfiled(.1,.8,80,.1,5,true);

        //Block is in Left Starting position

        //Bock is in Mid Starting Position

        encoderDriveProfiled(.1,.1,.7,27,1,7,true); //Move towards the block
        turnInCircleProfiled(10,.2,-1,90,-.1,-.1,-.7,.1,20); //back up to prepare going forward
        encoderDriveProfiled(.1,.3,.8,80,1,7,true); //start going towards the platform
        turnInCircleProfiled(10,.2,1,45,.3,.3,.5,1,10);
        turnInCircleProfiled(10,.2,-1,45,.3,.3,.5,1,10);
        //Block is in Right Starting Position

        targetsSkyStone.activate();

        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }


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

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    public double roundDouble(double x){
        DecimalFormat twoDForm = new DecimalFormat("0.########");
        String str = twoDForm.format(x);
        return Double.valueOf(str);
    }
    public double scaleSpeed (double maxSpeed, double minSpeed, double targetPos, double currentPos){
        if (firstTime){
            startPos = currentPos;
            firstTime = false;
            finished = false;
        }
        double speedDif = maxSpeed - minSpeed;
        double dis = targetPos - startPos;
        double midPt = startPos + dis/2;
        if(Math.abs(currentPos) >= Math.abs(startPos) && Math.abs(currentPos) <= Math.abs(midPt)){
            return minSpeed + ((currentPos- startPos)/(dis/2))*(speedDif);
        }
        else if (Math.abs(currentPos) <= Math.abs(targetPos) && Math.abs(currentPos) > Math.abs(midPt)){
            return minSpeed + ((targetPos - currentPos)/(dis/2))*(speedDif);
        }
        if (Math.abs(currentPos) >= Math.abs(targetPos)){
            firstTime = true;
            finished = true;
        }
        return 0;
    }
    public double scaleSpeed2 (double maxSpeed, double minSpeed, double targetPos, double currentPos){
        if (firstTime){
            startPos = currentPos;
            firstTime = false;
            finished = false;
        }
        //  if(Math.abs(currentPos - startPos) <= Math.abs(targetPos - startPos)) {
        double scale = 1 - (Math.abs(currentPos-startPos)) / (Math.abs(targetPos-startPos));
        return scale * (maxSpeed - minSpeed) + minSpeed;
        //  }
        //   return 0;
    }
    public void turnInCircle(double radius, double velocity, double turnDirection, double rotations, double timeout) {
        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        middleMotor.setMode(STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(STOP_AND_RESET_ENCODER);

        leftMotor.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        middleMotor.setMode(RUN_USING_ENCODER);
        middleMotor2.setMode(RUN_USING_ENCODER);

        boolean finishedMotion = false;
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double startingAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
        double leftPower = 0;
        double rightPower = 0;
        double leftRadius = 0;
        double rightRadius = 0;
        if(turnDirection < 0) { //counterclockwize is negative
            leftRadius = radius - 8.25;
            rightRadius = radius + 8.25;
            leftPower = (2 * velocity) / (1 + (rightRadius / leftRadius));
            rightPower = (rightRadius / leftRadius) * ((2 * velocity) / (1 + (rightRadius / leftRadius)));

            if(rightPower > 1) {
                leftPower = leftPower * (1 / rightPower);
                rightPower = 1;
            }
        } else {
            leftRadius = radius + 8.25;
            rightRadius = radius - 8.25;
            rightPower = (2 * velocity) / (1 + (leftRadius / rightRadius));
            leftPower = (leftRadius / rightRadius) * ((2 * velocity) / (1 + (leftRadius / rightRadius)));
            if(leftPower > 1) {
                rightPower = rightPower * (1 / leftPower);
                leftPower = 1;
            }
        }

        while(opModeIsActive() && !finishedMotion) {


            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            double currentAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
            double angleDifference = Math.max(currentAngle,startingAngle) - Math.min(currentAngle,startingAngle);
            if(angleDifference >= rotations) {
                finishedMotion = true;
            }

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Angle Diff", angleDifference);
            telemetry.update();

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void turnInCircleProfiled(double radius, double velocity, double turnDirection, double rotations, double minSpeedInitial, double minSpeedFinal, double maxSpeed, double speedUpAt, double slowDownAt) {
        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        middleMotor.setMode(STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(STOP_AND_RESET_ENCODER);

        leftMotor.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        middleMotor.setMode(RUN_USING_ENCODER);
        middleMotor2.setMode(RUN_USING_ENCODER);

        boolean finishedMotion = false;
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double startingAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
        double leftPower = 0;
        double rightPower = 0;
        double leftRatio;
        double rightRatio;
        double leftRadius;
        double rightRadius;
        if(turnDirection < 0) { //counterclockwize is negative
            leftRadius = radius - 8.25;
            rightRadius = radius + 8.25;
            leftRatio = (2 * velocity) / (1 + (rightRadius / leftRadius));
            rightRatio = (rightRadius / leftRadius) * ((2 * velocity) / (1 + (rightRadius / leftRadius)));

            if(rightPower > 1) {
                leftRatio = leftRatio * (1 / rightRatio);
                rightRatio = 1;
            }
        } else {
            leftRadius = radius + 8.25;
            rightRadius = radius - 8.25;
            rightRatio = 1 * (2 * velocity) / (1 + (leftRadius / rightRadius));
            leftRatio = 1 * (leftRadius / rightRadius) * ((2 * velocity) / (1 + (leftRadius / rightRadius)));
            if(leftPower > 1) {
                rightRatio = rightRatio * (1 / leftRatio);
                leftRatio = 1;
            }
        }
        double speedDifferenceInitial = maxSpeed - minSpeedInitial;
        minSpeedInitial = minSpeedInitial / (Math.max(Math.abs(leftRatio),Math.abs(rightRatio)));
        speedDifferenceInitial = speedDifferenceInitial / (Math.max(Math.abs(leftRatio),Math.abs(rightRatio)));
        double speedDifferenceFinal = maxSpeed - minSpeedFinal;
        minSpeedFinal = minSpeedFinal / (Math.max(Math.abs(leftRatio),Math.abs(rightRatio)));
        speedDifferenceFinal = speedDifferenceFinal / (Math.max(Math.abs(leftRatio),Math.abs(rightRatio)));
        while(opModeIsActive() && !finishedMotion) {
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            double currentAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
            double angleDifference = extraClasses.angleDistance(currentAngle,startingAngle);
            if(angleDifference >= rotations) {
                finishedMotion = true;
            }

            int currentState = 0;
            if(angleDifference <= speedUpAt) {
                currentState = 1;
                leftPower = (minSpeedInitial * leftRatio) + (leftRatio * (angleDifference / speedUpAt) * speedDifferenceInitial);
                rightPower = (minSpeedInitial * rightRatio) + (rightRatio * (angleDifference / speedUpAt) * speedDifferenceInitial);
            } else if (angleDifference > speedUpAt && angleDifference < (rotations - slowDownAt)) {
                currentState = 2;
                leftPower = (leftRatio * minSpeedInitial) + (leftRatio * speedDifferenceInitial);
                rightPower = (rightRatio * minSpeedInitial) + (rightRatio * speedDifferenceInitial);
            } else if(angleDifference > (rotations - slowDownAt)) {
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

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double currentAngle = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
        telemetry.addData("Angle", currentAngle);
        telemetry.update();
    }
    public void encoderDriveProfiled(double minSpeedInitial, double minSpeedFinal, double maxSpeed, double inches, double speedUpAt, double slowDownAt, boolean end) {
        int newSideTargets = 0;
        speedUpAt = speedUpAt * COUNTS_PER_INCH;
        slowDownAt = slowDownAt * COUNTS_PER_INCH;
        double speedDifferenceInitial = maxSpeed - minSpeedInitial;
        double speedDifferenceFinal = maxSpeed - minSpeedFinal;
        if(opModeIsActive()) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newSideTargets = /*robot.leftMotor.getCurrentPosition() + */(int) ((Math.abs(inches)) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newSideTargets);
            rightMotor.setTargetPosition(newSideTargets);

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double currentPowerLeft = 0;
            double currentPowerRight = 0;
            while(Math.abs(leftMotor.getCurrentPosition()) < newSideTargets && Math.abs(rightMotor.getCurrentPosition()) < newSideTargets) {
                int stateAt = 0;
                double currentEncoderPos = leftMotor.getCurrentPosition();
                if(Math.abs(currentEncoderPos) < Math.abs(speedUpAt)) {
                    stateAt = 1;
                    currentPowerLeft = minSpeedInitial + (speedDifferenceInitial * (Math.abs(currentEncoderPos / speedUpAt)));
                    currentPowerRight = minSpeedInitial + (speedDifferenceInitial * (Math.abs(currentEncoderPos / speedUpAt)));
                } else if(Math.abs(currentEncoderPos) >= Math.abs(speedUpAt) && Math.abs(currentEncoderPos) <= (newSideTargets - slowDownAt)) {
                    stateAt = 2;
                    currentPowerLeft = minSpeedInitial + speedDifferenceInitial;
                    currentPowerRight = minSpeedInitial + speedDifferenceInitial;
                } else if(Math.abs(currentEncoderPos) > (newSideTargets - slowDownAt)) {
                    stateAt = 3;
                    currentPowerLeft = minSpeedFinal + (speedDifferenceFinal * ((newSideTargets - Math.abs(currentEncoderPos)) / (newSideTargets - slowDownAt)));
                    currentPowerRight = minSpeedFinal + (speedDifferenceFinal * ((newSideTargets - Math.abs(currentEncoderPos)) / (newSideTargets - slowDownAt)));
                }

                leftMotor.setPower(currentPowerLeft);
                rightMotor.setPower(currentPowerRight);
                telemetry.addData("Power", currentPowerLeft);
                telemetry.addData("At", stateAt);
                telemetry.addData("Target", newSideTargets);
                telemetry.addData("Current Pos" , leftMotor.getCurrentPosition());
                telemetry.addData("Current Right", rightMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        if(end == true) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
    public void encoderDriveProfiledDeleteLater(double minSpeed, double maxSpeed, double inches, double speedUpAt, double slowDownAt, boolean end) {
        int newSideTargets = 0;
        if(opModeIsActive()) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newSideTargets = /*robot.leftMotor.getCurrentPosition() + */(int) ((Math.abs(inches)) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newSideTargets);
            rightMotor.setTargetPosition(newSideTargets);

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            double currentPowerLeft = 0;
            double currentPowerRight = 0;
            while(Math.abs(leftMotor.getCurrentPosition()) < newSideTargets && Math.abs(rightMotor.getCurrentPosition()) < newSideTargets) {
                int stateAt = 0;
                if((Math.abs(leftMotor.getCurrentPosition())) < (Math.abs(speedUpAt) * COUNTS_PER_INCH)) {
                    stateAt = 1;
                    currentPowerLeft = minSpeed + (maxSpeed * (Math.abs(leftMotor.getCurrentPosition())/(Math.abs(speedUpAt) * COUNTS_PER_INCH)));
                    currentPowerRight = minSpeed + (maxSpeed * (Math.abs(rightMotor.getCurrentPosition())/(Math.abs(speedUpAt) * COUNTS_PER_INCH)));
                } else if((newSideTargets - Math.abs(leftMotor.getCurrentPosition())) > (Math.abs(slowDownAt) * COUNTS_PER_INCH) && (Math.abs(leftMotor.getCurrentPosition())) > (Math.abs(speedUpAt) * COUNTS_PER_INCH)) {
                    currentPowerLeft = maxSpeed + minSpeed;
                    currentPowerRight = maxSpeed + minSpeed;
                    stateAt = 2;
                } else {
                    if(maxSpeed > 0) {
                        currentPowerLeft = minSpeed + ((maxSpeed) * ((Math.abs(newSideTargets) - Math.abs(leftMotor.getCurrentPosition())) / (Math.abs(slowDownAt) * COUNTS_PER_INCH)));
                        currentPowerRight = minSpeed + ((maxSpeed) * ((Math.abs(newSideTargets) - Math.abs(rightMotor.getCurrentPosition())) / (Math.abs(slowDownAt) * COUNTS_PER_INCH)));
                    }
                    else {
                        currentPowerLeft = -.08 + ((maxSpeed + minSpeed) * ((Math.abs(newSideTargets) - Math.abs(leftMotor.getCurrentPosition())) / (Math.abs(slowDownAt) * COUNTS_PER_INCH)));
                        currentPowerRight = -.08 + ((maxSpeed + minSpeed) * ((Math.abs(newSideTargets) - Math.abs(rightMotor.getCurrentPosition())) / (Math.abs(slowDownAt) * COUNTS_PER_INCH)));
                    }
                    stateAt = 3;
                }
                leftMotor.setPower(currentPowerLeft);
                rightMotor.setPower(currentPowerRight);
                telemetry.addData("Power", currentPowerLeft);
                telemetry.addData("At", stateAt);
                telemetry.addData("Target", newSideTargets);
                telemetry.addData("Current Pos" , leftMotor.getCurrentPosition());
                telemetry.addData("Current Right", rightMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        if(end == true) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
}


