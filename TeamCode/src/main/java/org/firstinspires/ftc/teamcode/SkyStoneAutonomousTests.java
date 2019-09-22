package org.firstinspires.ftc.teamcode;
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

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

/**
 * Created by Luke on 9/25/2016. Final Far Auto
 */
@Autonomous(name= "Turning While Moving", group = "HDrive")
public class SkyStoneAutonomousTests extends LinearOpMode {
    VuforiaLocalizer vuforia;
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();// Use a Pushbot's hardware
    static final double     COUNTS_PER_MOTOR_REV    = 1150 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4; ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = /*(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*/ 90.8;
    static final double COUNTS_PER_INCH_SIDE = 125;
    double initialAngle;
    boolean firstTime = true;
    double startPos;
    boolean finished = true;
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;

    BezierCurve curveDrive;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AUNOqaX/////AAABmRYrfwXgf0eBiFui4bMgqFOA9l1KyDSRCEC94Cy8lAqVsIBXI7SCxQxBhxloFpJ43BYXjodIylbiXiArCEhYbdiVvgI7iXO35pCSMnMLhSwZ7+YnogjWO4wfJEe+WNGEV3sx1sI9Y14W45etXXpo7KzyGC5Eq8WTXkxSiIe8bnkSn3aZ+L1CFgIyk6fDajfxvyYWaZBlxQDM8fxIyBfthI0EBtfplBFrJLxqq8tgUEDBPiVlYcUuspKPk5ew1u4cHDJvtxC1UkIsXpABs7kGkSvh5/nAp02Hd7yWlw3UUlOGKDlOit2guJZEC/iSJqR5oQsx/nqLJakwCd6Z1NZZFAtZ2u8Gc4NtDpw+LW4o6uMD";
    private TFObjectDetector tfod;
    double angleDouble = 0;
    Orientation angles;
    PIDFCoefficients pidStuff;
    BezierCurve curveAlgorithm;
    @Override public void runOpMode() throws InterruptedException {
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initVuforia();
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

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
        curveAlgorithm = new BezierCurve();

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
        curveDrive = new BezierCurve();
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
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) + 180;
        initialAngle = angleDouble;
        telemetry.addLine("Ready to Begin");
        telemetry.addData("Starting Angle", initialAngle);
        telemetry.update();

        waitForStart();
        moveBaseWhileTurning();

    }
    public double convertAngle(double angle) {
        double angleUsed = angle + 180;
        return angleUsed;
    }
    public double offsetAngle(double angle) {
        double newAngle = angle;
        if(angle > 360) {
            newAngle = angle - 360;
        }
        return newAngle;
    }
    public void moveBaseWhileTurning() {
        boolean thing = true;
        int loops = 0 ;
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) + 180;
        double lastLeftPos = leftMotor.getCurrentPosition();
        double lastRightPos = rightMotor.getCurrentPosition();
        double lastAngle = angleDouble;
        double xPos = 0;
        double yPos = 0;
        double previousXPos = 0;
        double previousYPos = 0;
        double arcLength = 0;
        double currentAngle = angleDouble;
        double t = 0;
        double leftPower = 0;
        double rightPower = 0;
        double totalPosChange = 0;
        while(thing && opModeIsActive()) {
            loops++;
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = offsetAngle(convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)))+180);
            double leftCurrentPos = leftMotor.getCurrentPosition();
            double rightCurrentPos = rightMotor.getCurrentPosition();
            double leftPosChange = leftCurrentPos - lastLeftPos;
            double rightPosChange = rightCurrentPos - lastRightPos;
            lastLeftPos = leftCurrentPos;
            lastRightPos = rightCurrentPos;
            previousXPos = xPos;
            previousYPos = yPos;
            double odometry = 360 - angleDouble;
            xPos = xPos + (((leftPosChange + rightPosChange)/2)/COUNTS_PER_INCH) * Math.sin(Math.toRadians(odometry));
            yPos = yPos + (((leftPosChange + rightPosChange)/2)/COUNTS_PER_INCH) * Math.cos(Math.toRadians(odometry));
            curveAlgorithm.calculateMovement(new Point(0,40), new Point(40,40),t);
            arcLength =  arcLength + ExtraClasses.distanceBetweenPoints(previousXPos, xPos, previousYPos, yPos);
            t = arcLength / 62.8;
            ;
            telemetry.addData("Left Encoder Ticks", leftMotor.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.addData("Right Encoder Ticks", rightMotor.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.addData("odometry", odometry);
            telemetry.addData("Predicted Angle", Math.toDegrees(Math.atan(curveAlgorithm.findDerivative(t))));
            telemetry.addData("Angle", angleDouble - 270);
            telemetry.addData("Actual angle", angleDouble);
            telemetry.addData("XPos", xPos);
            telemetry.addData("YPos", yPos);
            telemetry.addData("Goal X Pos", BezierCurve.returnXCord(t));
            telemetry.addData("Goal Y Pos", BezierCurve.returnYCord(t));
            telemetry.addData("t", t);
            telemetry.addData("Inverse Slope", curveAlgorithm.getInverseSlope());
            telemetry.addData("Left Distance", curveAlgorithm.getLeftDistance());
            telemetry.addData("Right Distance", curveAlgorithm.getRightDistance());
            telemetry.addData("Left Ratio", curveAlgorithm.getLeftRatio());
            telemetry.addData("Right Ratio", curveAlgorithm.getRightRatio());
            telemetry.addData("Actual Left", Math.min(leftMotor.getVelocity(),rightMotor.getVelocity())/(Math.max(leftMotor.getVelocity(),rightMotor.getVelocity())));
            telemetry.update();
            leftPower = curveAlgorithm.getLeftRatio() * .45;
            rightPower = curveAlgorithm.getRightDrive() * .45;
            if(t <= 1) {
                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower);
            }
            else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

        }
    }
    public void moveBaseAndArm(double sidePower, double midPower, double shoulderPower, double elbowPower, double rotationPower, double sideInches, double midInches, int shoulderTicks, int elbowTicks, int rotationTicks, Telemetry telemetry) {
        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        middleMotor.setMode(STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(STOP_AND_RESET_ENCODER);

        leftMotor.setMode(RUN_TO_POSITION);
        rightMotor.setMode(RUN_TO_POSITION);
        middleMotor.setMode(RUN_TO_POSITION);
        middleMotor2.setMode(RUN_TO_POSITION);

        //Establish Goal Values
        double sideTicks = sideInches * COUNTS_PER_INCH;
        double midTicks = midInches * COUNTS_PER_INCH_SIDE * 2;

        leftMotor.setTargetPosition((int)sideTicks);
        rightMotor.setTargetPosition((int)sideTicks);
        middleMotor.setTargetPosition((int)midTicks);
        middleMotor2.setTargetPosition((int)midTicks);

        leftMotor.setPower(sidePower);
        rightMotor.setPower(sidePower);
        middleMotor.setPower(midPower);
        middleMotor2.setPower(midPower);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) + 180;
        double startingAngle = angleDouble;
        double endingAngle = 0;
        double angleError = 0;
        int runThroughs = 0;
        boolean atHere = false;
        double sideChange = 0;
        while((leftMotor.isBusy() || rightMotor.isBusy() || middleMotor.isBusy() || middleMotor2.isBusy() && opModeIsActive())) {
            telemetry.addData("Left Position : Goal", leftMotor.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Right Position : Goal", rightMotor.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Middle Position : Goal", middleMotor.getCurrentPosition() + ":" + midTicks);
            telemetry.addData("Middle2 Position : Goal", middleMotor2.getCurrentPosition() + ":" + midTicks);
            telemetry.addData("Angle", angleDouble);
            telemetry.addData("At Here", sideChange);
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.addData("Is Busy Left", leftMotor.isBusy());
            telemetry.addData("Is Busy Mid ", middleMotor.isBusy());
            telemetry.addData("Is Busy Mid 2", middleMotor2.isBusy());
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) + 180;
            if(runThroughs > 2) {
                atHere = true;
                angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) + 180;
                endingAngle = angleDouble;
                angleError = endingAngle - startingAngle;
                sideChange = (angleError)/100;
                if(leftMotor.isBusy() && rightMotor.isBusy()) {
                    leftMotor.setPower(sidePower + sideChange);
                    rightMotor.setPower(sidePower - sideChange);
                }
                if(!rightMotor.isBusy() || !leftMotor.isBusy()) {
                    if(rightMotor.getMode() == RUN_TO_POSITION || leftMotor.getMode() == RUN_TO_POSITION) {
                        leftMotor.setMode(RUN_USING_ENCODER);
                        rightMotor.setMode(RUN_USING_ENCODER);
                    }
                    leftMotor.setPower(sideChange);
                    rightMotor.setPower(-sideChange);
                }

            }
            runThroughs++;
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        middleMotor.setPower(0);
        middleMotor2.setPower(0);

        leftMotor.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        middleMotor.setMode(RUN_USING_ENCODER);
        middleMotor2.setMode(RUN_USING_ENCODER);

        telemetry.addLine("Complete");
        telemetry.update();
    }

    public void realignRobot() {
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) + 180;
        double currentAngle = angleDouble;
        double sidePower = ((currentAngle - initialAngle) * 20)/100;
        while((angleDouble > (initialAngle + 1) || angleDouble < (initialAngle - 1)) && opModeIsActive()) {
            telemetry.addData("Current Angle", angleDouble);
            telemetry.update();
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) + 180;
            currentAngle = angleDouble;
            sidePower = ((currentAngle - initialAngle) * 6)/100;
            leftMotor.setPower(sidePower);
            rightMotor.setPower(-sidePower);
            telemetry.addData("initial Angle", initialAngle);
            telemetry.addData("current angle", currentAngle);
            telemetry.addData("Side Power", sidePower);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.maxWebcamAspectRatio = 10;
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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

}


