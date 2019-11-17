package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
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
@Autonomous(name= "Color Official Auto", group = "HDrive")
public class AutonomousOfficial extends LinearOpMode {
    AutoClasses extraClasses;
    VuforiaLocalizer vuforia;
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();// Use a Pushbot's hardware
    static final double     COUNTS_PER_MOTOR_REV    = 1150 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4; ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = /*(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*/ 91.125;
    static final double COUNTS_PER_INCH_SIDE = 125;
    double initialAngle;
    boolean firstTime = true;
    double startPos;
    boolean finished = true;
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx hangArm;
    DcMotorEx middleMotor2;
    DcMotorEx shoulderMotor;
    DcMotorEx elbowMotor;
    DcMotorEx rotationMotor;
    Servo leftIntake;
    Servo rightIntake;
    Servo bouncer;

    Servo hangArmLock;
    AutoCalculator calculator;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AUNOqaX/////AAABmRYrfwXgf0eBiFui4bMgqFOA9l1KyDSRCEC94Cy8lAqVsIBXI7SCxQxBhxloFpJ43BYXjodIylbiXiArCEhYbdiVvgI7iXO35pCSMnMLhSwZ7+YnogjWO4wfJEe+WNGEV3sx1sI9Y14W45etXXpo7KzyGC5Eq8WTXkxSiIe8bnkSn3aZ+L1CFgIyk6fDajfxvyYWaZBlxQDM8fxIyBfthI0EBtfplBFrJLxqq8tgUEDBPiVlYcUuspKPk5ew1u4cHDJvtxC1UkIsXpABs7kGkSvh5/nAp02Hd7yWlw3UUlOGKDlOit2guJZEC/iSJqR5oQsx/nqLJakwCd6Z1NZZFAtZ2u8Gc4NtDpw+LW4o6uMD";
    private TFObjectDetector tfod;
    String angleDouble = "0";
    Orientation angles;
    PIDFCoefficients pidStuff;
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
        hangArm = (DcMotorEx) hardwareMap.get(DcMotor.class, "Hang Arm");
        shoulderMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Shoulder Motor");
        elbowMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"Elbow Motor");
        rotationMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "Rotation Motor");
        leftIntake = hardwareMap.get(Servo.class, "Left Intake");
        rightIntake = hardwareMap.get(Servo.class, "Right Intake");
        bouncer = hardwareMap.get(Servo.class, "Bouncer");
        hangArmLock = hardwareMap.servo.get("Hang Arm Lock");

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
        hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition(),
                middleMotor.getCurrentPosition());
        telemetry.update();
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor2.setDirection(DcMotor.Direction.REVERSE);
        elbowMotor.setDirection(DcMotor.Direction.REVERSE);
        calculator = new AutoCalculator();
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
        hangArm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        initialAngle = Double.parseDouble(angleDouble);
        telemetry.addLine("Ready to Begin");
        telemetry.addData("Starting Angle", initialAngle);
        telemetry.update();
        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extraClasses = new AutoClasses(leftMotor, rightMotor, middleMotor, middleMotor2, shoulderMotor, elbowMotor, rotationMotor, imu);


        waitForStart();
        releaseArm();
        realignRobot();
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        telemetry.addData("Current Angle", angleDouble);
        telemetry.update();

        tfod.activate();
        int objectsFound = 0;
        int position = 0;
        while(objectsFound != 3) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    objectsFound = updatedRecognitions.size();
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            telemetry.addData("X1",goldMineralX);
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                position = 1;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                position = 3;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                position = 2;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        if(position == 1) {
            telemetry.addLine("Should be Left");
            moveBaseAndArm(.6, -.45, 0.1, .1, .4, 36, 9,0,350,0, telemetry);
            moveArmToPos(550,2800,-1800,.7,.7,.6);
            moveArmToPos(-1300,2800,-1800,-.7,.7,.1);
            moveArmToPos(-1458, 2362, -1800, .7, .7, -.6);
            moveArmToPos(0, 300, 0, .7, .7, -.3);
            leftMotor.setMode(RUN_USING_ENCODER);
            rightMotor.setMode(RUN_USING_ENCODER);
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            while(Double.parseDouble(angleDouble) > -132 && opModeIsActive()) {
                telemetry.addData("angle", angleDouble);
                telemetry.addData("power", scaleSpeed(.8, .2, -135, Double.parseDouble(angleDouble)));
                telemetry.addData("in loop", 5);
                telemetry.update();
                angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
                leftMotor.setPower(scaleSpeed2(.8, .2, -135, Double.parseDouble(angleDouble)));
                rightMotor.setPower(scaleSpeed2(-.8, -.2, -135, Double.parseDouble(angleDouble)));
            }
            moveBaseAndArm(.1,-.5,.7,.7,.1,0,-10,0,0,0,telemetry);
            moveBaseAndArm(.5,-.1,.7,.7,-.5,-6,0,2963,-700,-3100,telemetry);
            leftMotor.setMode(RUN_WITHOUT_ENCODER);
            rightMotor.setMode(RUN_WITHOUT_ENCODER);
            while(opModeIsActive()){
                leftMotor.setPower(-.2);
                rightMotor.setPower(-.2);
            }
            telemetry.update();


        }
        else if(position == 2) {
            telemetry.addLine("Should be middle");
            telemetry.update();
            moveBaseAndArm(.6,0,.4,.2,-.3,30,0,2700,500,-1000,telemetry);
            moveArmToPos(-1200,2700,-1000,-.4,.1,.1);
            moveBaseAndArm(-.5,0,0.7,0.5,0.5,-14,0,300,0,0,telemetry);
            moveBaseAndArm(0,.8,.1,.1,.1,0,14,0,0,0,telemetry);
            leftMotor.setMode(RUN_USING_ENCODER);
            rightMotor.setMode(RUN_USING_ENCODER);
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            while(Double.parseDouble(angleDouble) > -132 && opModeIsActive()){
                telemetry.addData("angle", angleDouble);
                telemetry.addData("power", scaleSpeed(.8,.2,-135, Double.parseDouble(angleDouble)));
                telemetry.update();
                angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
                leftMotor.setPower(scaleSpeed2(.8,.2,-135, Double.parseDouble(angleDouble)));
                rightMotor.setPower(scaleSpeed2(-.8,-.2,-135, Double.parseDouble(angleDouble)));
            }
            moveBaseAndArm(.7,0,.7,.7,-.5,-15,0,2963,-700,-3100,telemetry);
            leftMotor.setMode(RUN_WITHOUT_ENCODER);
            rightMotor.setMode(RUN_WITHOUT_ENCODER);
            while(opModeIsActive()){
                leftMotor.setPower(-.2);
                rightMotor.setPower(-.2);
            }
            telemetry.update();

        }
        else if(position == 3) {
            telemetry.addLine("Should be Right");
            moveBaseAndArm(.3,-.5,.1,.1,.7,36,-15.27,0,350,0,telemetry);
            moveArmToPos(550,3300,0,.7,.7,.6);
            moveArmToPos(-1300,3300,0,-.7,.1,.1);
            moveArmToPos(0, 300, 0, .7, .7, -.1);
            middleMotor2.setTargetPositionTolerance(20);
            middleMotor.setTargetPositionTolerance(20);
            moveBaseAndArm(-.7,.8,0.1,0.1,0.1,-19,14,0,0,0,telemetry);
            moveBaseAndArm(0.1,.8,.1,.1,.1,0,16,0,0,0,telemetry);
            leftMotor.setMode(RUN_USING_ENCODER);
            rightMotor.setMode(RUN_USING_ENCODER);
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            while(Double.parseDouble(angleDouble) > -132 && opModeIsActive()){
                telemetry.addData("angle", angleDouble);
                telemetry.addData("power", scaleSpeed(.8,.2,-135, Double.parseDouble(angleDouble)));
                telemetry.update();
                angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
                leftMotor.setPower(scaleSpeed2(.8,.2,-135, Double.parseDouble(angleDouble)));
                rightMotor.setPower(scaleSpeed2(-.8,-.2,-135, Double.parseDouble(angleDouble)));
            }
            moveBaseAndArm(.7,0,.7,.7,-.5,-12,0,2963,-700,-3100,telemetry);
            leftMotor.setMode(RUN_WITHOUT_ENCODER);
            rightMotor.setMode(RUN_WITHOUT_ENCODER);
            while(opModeIsActive()){
                leftMotor.setPower(-.2);
                rightMotor.setPower(-.2);
            }
            telemetry.update();
        }
        else {
            telemetry.addLine("Yea you broke something lmao");
            telemetry.update();
        }
        //140


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
    public void moveBaseAndArm(double sidePower, double midPower, double shoulderPower, double elbowPower, double rotationPower, double sideInches, double midInches, int shoulderTicks, int elbowTicks, int rotationTicks, Telemetry telemetry) {
        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        middleMotor.setMode(STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(STOP_AND_RESET_ENCODER);

        leftMotor.setMode(RUN_TO_POSITION);
        rightMotor.setMode(RUN_TO_POSITION);
        middleMotor.setMode(RUN_TO_POSITION);
        middleMotor2.setMode(RUN_TO_POSITION);
        shoulderMotor.setMode(RUN_TO_POSITION);
        elbowMotor.setMode(RUN_TO_POSITION);
        rotationMotor.setMode(RUN_TO_POSITION);

        //Establish Goal Values
        double sideTicks = sideInches * COUNTS_PER_INCH;
        double midTicks = midInches * COUNTS_PER_INCH_SIDE * 2;

        leftMotor.setTargetPosition((int)sideTicks);
        rightMotor.setTargetPosition((int)sideTicks);
        middleMotor.setTargetPosition((int)midTicks);
        middleMotor2.setTargetPosition((int)midTicks);
        shoulderMotor.setTargetPosition(shoulderTicks);
        elbowMotor.setTargetPosition(elbowTicks);
        rotationMotor.setTargetPosition(rotationTicks);

        leftMotor.setPower(sidePower);
        rightMotor.setPower(sidePower);
        middleMotor.setPower(midPower);
        middleMotor2.setPower(midPower);
        shoulderMotor.setPower(shoulderPower);
        elbowMotor.setPower(elbowPower);
        rotationMotor.setPower(rotationPower);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        boolean elbowFirstTime = true;
        int elbowPosKeep = 0;
        boolean shoulderFirstTime = true;
        int shoulderPosKeep = 0;
        double startingAngle = Double.parseDouble(angleDouble);
        double endingAngle = 0;
        double angleError = 0;
        int runThroughs = 0;
        boolean atHere = false;
        double sideChange = 0;
        while((leftMotor.isBusy() || rightMotor.isBusy() || middleMotor.isBusy() || middleMotor2.isBusy() || shoulderMotor.isBusy() || elbowMotor.isBusy() || rotationMotor.isBusy())&&opModeIsActive()) {
            telemetry.addData("Left Position : Goal", leftMotor.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Right Position : Goal", rightMotor.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Middle Position : Goal", middleMotor.getCurrentPosition() + ":" + midTicks);
            telemetry.addData("Middle2 Position : Goal", middleMotor2.getCurrentPosition() + ":" + midTicks);
            telemetry.addData("Shoulder Position : Goal", shoulderMotor.getCurrentPosition() + ":" + shoulderTicks);
            telemetry.addData("Elbow Position : Goal", elbowMotor.getCurrentPosition() + ":" + elbowTicks);
            telemetry.addData("Rotation Position : Goal", rotationMotor.getCurrentPosition() + ":" + rotationTicks);
            telemetry.addData("Angle", angleDouble);
            telemetry.addData("At Here", sideChange);
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.addData("Is Busy Left", leftMotor.isBusy());
            telemetry.addData("Is Busy Mid ", middleMotor.isBusy());
            telemetry.addData("Is Busy Mid 2", middleMotor2.isBusy());
            telemetry.addData("Is Busy Elbow ", elbowMotor.isBusy());
            telemetry.addData("Is Busy Shoulder", shoulderMotor.isBusy());
            telemetry.addData("Is Busy Rotation", rotationMotor.isBusy());
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            if(runThroughs > 2) {
                atHere = true;
                angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
                endingAngle = Double.parseDouble(angleDouble);
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
            if(!elbowMotor.isBusy()) {
                if(elbowFirstTime) {
                    elbowFirstTime = false;
                    elbowPosKeep = elbowMotor.getCurrentPosition();
                }
                elbowMotor.setTargetPosition(elbowPosKeep);
                elbowMotor.setPower(.2);
                telemetry.addLine("Holding Elbow Pos");
            }
            if(!shoulderMotor.isBusy()) {
                if(shoulderFirstTime) {
                    shoulderFirstTime = false;
                    shoulderPosKeep = shoulderMotor.getCurrentPosition();
                }
                shoulderMotor.setTargetPosition(shoulderPosKeep);
                shoulderMotor.setPower(.2);
                telemetry.addLine("Holding Shoulder Pos");
            }
            runThroughs++;
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        middleMotor.setPower(0);
        middleMotor2.setPower(0);
        shoulderMotor.setPower(0);
        elbowMotor.setPower(0);
        rotationMotor.setPower(0);

        leftMotor.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        middleMotor.setMode(RUN_USING_ENCODER);
        middleMotor2.setMode(RUN_USING_ENCODER);
        shoulderMotor.setMode(RUN_USING_ENCODER);
        elbowMotor.setMode(RUN_USING_ENCODER);
        rotationMotor.setMode(RUN_USING_ENCODER);

        telemetry.addLine("Complete");
        telemetry.update();
    }
    public void releaseArm() throws InterruptedException {
        hangArm.setMode(RUN_WITHOUT_ENCODER);
        hangArm.setPower(0);
        hangArmLock.setPosition(.35);
        //hangArm.setPower(0);
        Thread.sleep(500);
        hangArm.setPower(0);

        //hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(hangArm.getCurrentPosition() > -5130 && opModeIsActive()){
            telemetry.addData("Hang Arm Pos", hangArm.getCurrentPosition());
            telemetry.update();
        }
        hangArm.setMode(RUN_USING_ENCODER);
        hangArm.setPower(0);
        middleMotor.setMode(RUN_TO_POSITION);
        middleMotor2.setMode(RUN_TO_POSITION);
        rotationMotor.setMode(RUN_TO_POSITION);
        //
        middleMotor.setTargetPosition(400);
        middleMotor2.setTargetPosition(400);
        rotationMotor.setTargetPosition(-2000);
        middleMotor.setPower(.6);
        middleMotor2.setPower(.6);
        rotationMotor.setPower(-.7);
        while((middleMotor.isBusy() || middleMotor2.isBusy() || rotationMotor.isBusy()) && opModeIsActive()) {
            telemetry.addData("Middle Motor", middleMotor.getCurrentPosition());
            telemetry.addData("Rotation Motor", rotationMotor.getCurrentPosition());
            telemetry.update();

        }
        middleMotor.setPower(0);
        middleMotor2.setPower(0);
        rotationMotor.setPower(0);
        middleMotor.setMode(RUN_USING_ENCODER);
        middleMotor2.setMode(RUN_USING_ENCODER);
        rotationMotor.setMode(RUN_USING_ENCODER);

    }
    public void realignRobot() {
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        double currentAngle = Double.parseDouble(angleDouble);
        double sidePower = ((currentAngle - initialAngle) * 20)/100;
        while((Double.parseDouble(angleDouble) > (initialAngle + 1) || Double.parseDouble(angleDouble) < (initialAngle - 1)) && opModeIsActive()) {
            telemetry.addData("Current Angle", angleDouble);
            telemetry.update();
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            currentAngle = Double.parseDouble(angleDouble);
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
    public void moveArmToPos(double elbowPos, double shoulderPos, double rotationPos, double elbowPower, double shoulderPower, double rotationPower) {
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setTargetPosition((int)elbowPos);
        shoulderMotor.setTargetPosition((int)shoulderPos);
        rotationMotor.setTargetPosition((int)rotationPos);
        elbowMotor.setPower(elbowPower);
        shoulderMotor.setPower(shoulderPower);
        rotationMotor.setPower(rotationPower);
        while((elbowMotor.isBusy() || shoulderMotor.isBusy() || rotationMotor.isBusy()) && opModeIsActive()) {
            telemetry.addData("Shoulder Position : Goal", shoulderMotor.getCurrentPosition() + ":" + shoulderPos);
            telemetry.addData("Elbow Position : Goal", elbowMotor.getCurrentPosition() + ":" + elbowPos);
            telemetry.addData("Rotation Position : Goal", rotationMotor.getCurrentPosition() + ":" + rotationPos);
            telemetry.update();
        }
        elbowMotor.setPower(0);
        shoulderMotor.setPower(0);
        rotationMotor.setPower(0);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("We Here");
        telemetry.update();

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


