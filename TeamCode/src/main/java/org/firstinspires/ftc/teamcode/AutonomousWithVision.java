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
 * Created by Luke on 9/25/2016.
 */
@Autonomous(name= "Color Far Auto ", group = "HDrive")
@Disabled()
public class AutonomousWithVision extends LinearOpMode {
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
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUNOqaX/////AAABmRYrfwXgf0eBiFui4bMgqFOA9l1KyDSRCEC94Cy8lAqVsIBXI7SCxQxBhxloFpJ43BYXjodIylbiXiArCEhYbdiVvgI7iXO35pCSMnMLhSwZ7+YnogjWO4wfJEe+WNGEV3sx1sI9Y14W45etXXpo7KzyGC5Eq8WTXkxSiIe8bnkSn3aZ+L1CFgIyk6fDajfxvyYWaZBlxQDM8fxIyBfthI0EBtfplBFrJLxqq8tgUEDBPiVlYcUuspKPk5ew1u4cHDJvtxC1UkIsXpABs7kGkSvh5/nAp02Hd7yWlw3UUlOGKDlOit2guJZEC/iSJqR5oQsx/nqLJakwCd6Z1NZZFAtZ2u8Gc4NtDpw+LW4o6uMD";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        telemetry.addData(">", "created by the best programmer adam");
        telemetry.addData(">", "Press play to start");
        telemetry.update();*/

        /*BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);*/

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
        //glyphMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Hirsh is very dumb");
        /*claw1 = hardwareMap.servo.get("claw1");
        claw2 = hardwareMap.servo.get("claw2");
        RelicClaw = hardwareMap.get(Servo.class, "Relic Claw");
        RelicClaw2 = hardwareMap.get(Servo.class, "Relic Claw 2");
        RelicArm = hardwareMap.get(DcMotor.class, "Relic Arm");*/

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


        moveArmToPos(350,0,0,.5,.1,.1);
        releaseArm();
        realignRobot();

        //releaseArm();
        //moveBaseAndArm(.1,.4,.1,.2,.6,0,3.2,0,500,-1580,telemetry);
        //moveBaseAndArm(.5,.4,.6,.3,.4,28,11,3300,500,-950,telemetry);
        //moveArmToPos(-1150,3300,-950,-.4,.1,.1);
        //releaseArm();
        //adjustRotationMotor();


        //extraClasses.moveBaseAndArm(.5,0,.4,.2,-.3,28,0,2650,500,-800,telemetry);
        //moveArmToPos(-1150,2700,-800,-.4,.1,.1);

        //extraClasses = new AutoClasses(leftMotor, rightMotor, middleMotor, middleMotor2, shoulderMotor, elbowMotor, rotationMotor, imu);
        //extraClasses.moveBaseAndArm(-.4,.1,-.5,.4,-.1,19,0,150,30,-800,telemetry);
        //moveArmToPos(-346,3771,-1000,-.2,.5,.3);
        //moveArmToPos(1000,1000,0,.3,.3,0);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        telemetry.addData("Current Angle", angleDouble);
        telemetry.update();
        //realignRobot();



        tfod.activate();
        int objectsFound = 0;
        int position = 0;
        while(objectsFound != 3 && opModeIsActive()) {
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
                                /*goldMineralX2 = (int) recognition.getRight();
                                goldMineralY = (int) recognition.getTop();
                                goldMineralY2 = (int) recognition.getBottom();*/
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
            //encoderDriveBoth(.6,.7,15,28,28,60);
            moveBaseAndArm(.6, .45, 0.1, .1, .4, 30, 10,0,350,0, telemetry);
            moveArmToPos(350,2850,-1200,.3,.6,.4);
            //moveBaseAndArm(.6,.7,.6,.3,.4,28,15,2850,350,-400,telemetry);
            moveArmToPos(-1300,2850,-1200,-.4,.1,.1);
            moveArmToPos(0, 0, -500, .7, .4, .1);
            /*//encoderDriveBoth(.5, .5, 27, 27, 27, 60);
            encoderDriveProfiledBoth(.1,.1,.5,.7,27,19,7,7,5,7,true);
            //encoderDriveProfiledBoth(.1,.1,.5,.7,27,19,7,7,5,7,true);
            telemetry.addLine("Completed");
            telemetry.update();




            //encoderDriveProfiledMiddle(-.1,-.4,-5,-3,true);
            //encoderDriveProfiledBoth(-.1,-.1,-.5,-.7,-10,-22,-4,-7,-4,-7,true);
            //encoderDriveBoth(.5,.5,-12,-12,-12,60);
            //Thread.sleep(100);
            //encoderDriveProfiledMiddle(-.1,-.8,-44,-7,true);
            //encoderDriveMiddle(.5,-60,60);
            //Thread.sleep(100);

            //turn2();
            //Thread.sleep(100);
            //Thread.sleep(100);
            //encoderDriveProfiledMiddle(.1,.4,8,2,true);
            //encoderDrive(.4,8,8,60);
            //encoderDriveBoth(.5, .5,5.3, -8.5,-8.5,60);
            //Thread.sleep(100);
            //encoderDriveBoth(.8, .5,0,10,10,60);*/
        }
        else if(position == 2) {
            telemetry.addLine("Should be middle");
            telemetry.update();
            moveBaseAndArm(.5,0,.4,.2,-.3,28,0,2650,500,-800,telemetry);
            moveArmToPos(-1300,2700,-800,-.4,.1,.1);

            moveBaseAndArm(-.4,.1,-.5,.4,-.1,19,0,150,30,-800,telemetry);
            /*//encoderDrive(.5,45,45,60);
            encoderDriveProfiled(.1,.8,45,16,10,true);
            //Thread.sleep(100);

            encoderDriveProfiled(-.1,-.8,-29,-16,6,true);
            //Thread.sleep(100);

            //encoderDriveMiddle(.5,-46,60);
            encoderDriveProfiledMiddle(-.1,-.8,-40,-8,true);*/
            moveArmToPos(0, 0, -500, .7, .4, .1);

            /*turn2();
            //Thread.sleep(100);
            //Thread.sleep(100);
            encoderDriveProfiledMiddle(.1,.4,6,1,true);
            //encoderDriveBoth(.4, .5,10,-8,-8,60);
            //Thread.sleep(100);
            encoderDrive(.4,7,7,60);*/
        }
        else if(position == 3) {
            telemetry.addLine("Should be Right");
            moveBaseAndArm(.3,-.5,.1,.1,.7,33,-14,0,350,0,telemetry);
            //encoderDriveBoth(.4,-.5,-14,33,33,60);
            moveArmToPos(550,3000,-150,.7,.7,.6);
            //moveBaseAndArm(.4,-.5,.4,.3,.2,33,-14,3000,550,-500,telemetry);
            moveArmToPos(-1300,2700,-150,-.7,.1,.1);
            //encoderDriveProfiledBoth(.1,-.1,.6,-.6,33,-30,10,-10,7,-5,true);
            //encoderDriveBoth(.5, .5, -29, 29, 29, 60);
            //encoderDriveBoth(.5,.5,29,29,29,60);
            //Thread.sleep(100);
            //Thread.sleep(100);
            moveArmToPos(0, 0, -500, .7, .4, .1);
            //turn2();
            //encoderDriveBoth(-.41,-.43,-25.9,-27.7,-27.7,60);
            //encoderDriveProfiled(.1,.7,35,7,6,true);
            telemetry.update();
        }
        else {
            telemetry.addLine("Yea you broke something lmao");
            telemetry.update();
        }
        //140


    }
    public void turn() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double finalAngle = 0;
        double power = 0;
        double realAngle = convertAngle(angleDouble);
        while(Double.parseDouble(angleDouble) > 168 || Double.parseDouble(angleDouble) < 10) {
            double maxPower = .4;
            double minPower = .04;//232 to 170
            power = minPower + (maxPower * (((realAngle-170))/62));

            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            telemetry.addData("updated Angle", realAngle);
            telemetry.addData("power", power);
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            realAngle = convertAngle(angleDouble);
            //telemetry.addData("left power",leftMotor.getPower());
            telemetry.update();
            try{
                Thread.sleep(1);
            }
            catch(InterruptedException e){

            }
            finalAngle = Double.parseDouble(angleDouble);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        try{
            Thread.sleep(1000);
        }
        catch(InterruptedException e){

        }
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        telemetry.addData("finalAngle", finalAngle);
        telemetry.addData("Current Angle", angleDouble);
        telemetry.update();
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
    public void turn2() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPower(.3);
        rightMotor.setPower(-.3);
        double finalAngle = 0;
        while(Double.parseDouble(angleDouble) > -133 && Double.parseDouble(angleDouble) < 10) {
            double maxPower = .6;
            double minPower = .1;
            double power = minPower + (maxPower * ((133-Math.abs(Double.parseDouble(angleDouble)))/133));
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            telemetry.addData("angle", angleDouble);
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            telemetry.addData("left power",leftMotor.getPower());
            telemetry.update();
            try{
                Thread.sleep(1);
            }
            catch(InterruptedException e){

            }
            finalAngle = Double.parseDouble(angleDouble);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        telemetry.addLine("Done");
        telemetry.update();
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
    }
    public void encoderDriveProfiled(double minSpeed, double maxSpeed, double inches, double slowDownAt, double speedUpAt, boolean end) {
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
                if((newSideTargets - Math.abs(leftMotor.getCurrentPosition())) > (Math.abs(slowDownAt) * COUNTS_PER_INCH) && (Math.abs(leftMotor.getCurrentPosition())) > (Math.abs(speedUpAt) * COUNTS_PER_INCH)) {
                    currentPowerLeft = maxSpeed + minSpeed;
                    currentPowerRight = maxSpeed + minSpeed;
                    stateAt = 1;
                }
                else if((Math.abs(leftMotor.getCurrentPosition())) < (Math.abs(speedUpAt) * COUNTS_PER_INCH)) {
                    stateAt = 3;
                    currentPowerLeft = minSpeed + (maxSpeed * (Math.abs(leftMotor.getCurrentPosition())/(Math.abs(speedUpAt) * COUNTS_PER_INCH)));
                    currentPowerRight = minSpeed + (maxSpeed * (Math.abs(rightMotor.getCurrentPosition())/(Math.abs(speedUpAt) * COUNTS_PER_INCH)));
                }
                else {
                    if(maxSpeed > 0) {
                        currentPowerLeft = minSpeed + ((maxSpeed) * ((Math.abs(newSideTargets) - Math.abs(leftMotor.getCurrentPosition())) / (Math.abs(slowDownAt) * COUNTS_PER_INCH)));
                        currentPowerRight = minSpeed + ((maxSpeed) * ((Math.abs(newSideTargets) - Math.abs(rightMotor.getCurrentPosition())) / (Math.abs(slowDownAt) * COUNTS_PER_INCH)));
                    }
                    else {
                        currentPowerLeft = -.05 + ((maxSpeed + minSpeed) * ((Math.abs(newSideTargets) - Math.abs(leftMotor.getCurrentPosition())) / (Math.abs(slowDownAt) * COUNTS_PER_INCH)));
                        currentPowerRight = -.05 + ((maxSpeed + minSpeed) * ((Math.abs(newSideTargets) - Math.abs(rightMotor.getCurrentPosition())) / (Math.abs(slowDownAt) * COUNTS_PER_INCH)));
                    }
                    stateAt = 2;
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
    public void encoderDriveProfiledMiddle(double minSpeed, double maxSpeed, double inches, double slowDownAt, boolean end) {
        int newMiddleTargets = 0;
        double startingAngle = 0;
        double endingAngle = 0;
        double angleError = 0;
        if(opModeIsActive()) {
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            startingAngle = Double.parseDouble(angleDouble);
            middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newMiddleTargets = /*robot.leftMotor.getCurrentPosition() + */(int) ((Math.abs(inches)) * COUNTS_PER_INCH_SIDE);
            middleMotor.setTargetPosition(newMiddleTargets);
            middleMotor2.setTargetPosition(newMiddleTargets);

            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            middleMotor.setPower(minSpeed);
            middleMotor2.setPower(minSpeed);

            double currentPower = 0;
            int runThroughs = 0;
            double sidePower = 0;
            boolean atHere = false;
            while(Math.abs(middleMotor.getCurrentPosition()) < newMiddleTargets && Math.abs(middleMotor2.getCurrentPosition()) < newMiddleTargets) {
                int stateAt = 0;
                if((newMiddleTargets - Math.abs(middleMotor.getCurrentPosition())) > (Math.abs(slowDownAt) * COUNTS_PER_INCH_SIDE)) {
                    currentPower = maxSpeed + minSpeed;
                    stateAt = 1;
                }
                else {
                    currentPower = minSpeed + (maxSpeed * ((Math.abs(newMiddleTargets) - Math.abs(middleMotor.getCurrentPosition()))/(Math.abs(slowDownAt) * COUNTS_PER_INCH_SIDE)));
                    stateAt = 2;
                }
                if(runThroughs > 8) {
                    atHere = true;
                    angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                    angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
                    endingAngle = Double.parseDouble(angleDouble);
                    angleError = endingAngle - startingAngle;
                    sidePower = (angleError * 4)/100;
                }
                leftMotor.setPower(sidePower);
                rightMotor.setPower(-sidePower);
                middleMotor.setPower(currentPower);
                middleMotor2.setPower(currentPower);
                telemetry.addData("Power", currentPower);
                telemetry.addData("At", stateAt);
                telemetry.addData("Target", newMiddleTargets);
                telemetry.addData("Current Pos" , middleMotor.getCurrentPosition());
                telemetry.addData("Current Right", middleMotor2.getCurrentPosition());
                telemetry.addData("Adjusting", atHere);
                telemetry.addData("Angle Error", angleError);
                telemetry.update();
                runThroughs++;
            }
        }
        if(end == true) {
            middleMotor.setPower(0);
            middleMotor2.setPower(0);
        }
    }
    public void encoderDriveProfiledBoth(double minSpeedSide, double minSpeedMiddle, double maxSpeedSide, double maxSpeedMiddle, double inchesSide, double inchesMiddle, double slowDownAtSide, double slowDownAtMiddle, double speedUpAtSide, double speedUpAtMiddle, boolean end) {
        int newSideTargets = 0;
        int newMiddleTargets = 0;
        double startingAngle = 0;
        double endingAngle = 0;
        double angleError = 0;
        if(opModeIsActive()) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newSideTargets = /*robot.leftMotor.getCurrentPosition() + */(int) ((Math.abs(inchesSide)) * COUNTS_PER_INCH);
            newMiddleTargets = (int) ((Math.abs(inchesMiddle)) * COUNTS_PER_INCH_SIDE);

            leftMotor.setTargetPosition(newSideTargets);
            rightMotor.setTargetPosition(newSideTargets);
            middleMotor.setTargetPosition(newMiddleTargets);
            middleMotor2.setTargetPosition(newMiddleTargets);

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            leftMotor.setPower(minSpeedSide);
            rightMotor.setPower(minSpeedSide);
            middleMotor.setPower(minSpeedMiddle);
            middleMotor2.setPower(minSpeedMiddle);

            double currentPowerSide = 0;
            double currentPowerMiddle = 0;
            int runThroughs = 0;
            double sideChangePower = 0;
            while(Math.abs(leftMotor.getCurrentPosition()) < newSideTargets || Math.abs(rightMotor.getCurrentPosition()) < newSideTargets || Math.abs(middleMotor.getCurrentPosition()) < newMiddleTargets || Math.abs(middleMotor2.getCurrentPosition()) < newMiddleTargets) {
                boolean atHere = false;
                int stateAt = 0;
                int stateAtMid = 0;
                if((newSideTargets - Math.abs(leftMotor.getCurrentPosition())) > (Math.abs(slowDownAtSide) * COUNTS_PER_INCH)&& (Math.abs(leftMotor.getCurrentPosition())) > (Math.abs(speedUpAtSide) * COUNTS_PER_INCH)) {
                    currentPowerSide = maxSpeedSide+minSpeedMiddle;
                    stateAt = 1;
                }
                else if((Math.abs(leftMotor.getCurrentPosition())) < (Math.abs(speedUpAtSide) * COUNTS_PER_INCH)) {
                    stateAt = 3;
                    currentPowerSide = minSpeedSide + (maxSpeedSide * (Math.abs(leftMotor.getCurrentPosition())/(Math.abs(speedUpAtSide) * COUNTS_PER_INCH)));
                }
                else if((Math.abs(leftMotor.getCurrentPosition())) > (Math.abs(newSideTargets))) {
                    currentPowerSide = 0;
                    stateAt = 4;
                }
                else {
                    currentPowerSide = minSpeedSide + (maxSpeedSide * ((Math.abs(newSideTargets) - Math.abs(leftMotor.getCurrentPosition()))/(Math.abs(slowDownAtSide) * COUNTS_PER_INCH)));
                    stateAt = 2;
                }

                if(newMiddleTargets - Math.abs(middleMotor.getCurrentPosition()) > (Math.abs(slowDownAtMiddle) * COUNTS_PER_INCH_SIDE) && (Math.abs(middleMotor.getCurrentPosition())) > (Math.abs(speedUpAtMiddle) * COUNTS_PER_INCH_SIDE)) {
                    currentPowerMiddle = maxSpeedMiddle+minSpeedMiddle;
                    stateAtMid = 1;
                }
                else if((Math.abs(middleMotor.getCurrentPosition())) < (Math.abs(speedUpAtMiddle) * COUNTS_PER_INCH_SIDE)) {
                    stateAtMid = 3;
                    currentPowerMiddle = minSpeedMiddle + (maxSpeedMiddle * (Math.abs(middleMotor.getCurrentPosition())/(Math.abs(speedUpAtMiddle) * COUNTS_PER_INCH_SIDE)));
                }
                else if((Math.abs(middleMotor.getCurrentPosition()) > Math.abs(newMiddleTargets))) {
                    currentPowerMiddle = 0;
                    stateAtMid = 4;
                }
                else {
                    currentPowerMiddle = minSpeedMiddle + (maxSpeedMiddle * (((Math.abs(newMiddleTargets) - Math.abs(middleMotor.getCurrentPosition()))/((Math.abs(slowDownAtMiddle)) * COUNTS_PER_INCH_SIDE))));
                    stateAtMid = 2;
                }

                if(runThroughs > 8) {
                    atHere = true;
                    angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                    angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
                    endingAngle = Double.parseDouble(angleDouble);
                    angleError = endingAngle - startingAngle;
                    sideChangePower = (angleError * 3)/100;
                }
                leftMotor.setPower(currentPowerSide + sideChangePower);
                rightMotor.setPower(currentPowerSide - sideChangePower);
                middleMotor.setPower(currentPowerMiddle);
                middleMotor2.setPower(currentPowerMiddle);
                telemetry.addData("Angle Error", Double.parseDouble(angleDouble) - startingAngle);
                telemetry.addData("Power Side", currentPowerSide);
                telemetry.addData("Power Middle", currentPowerMiddle);
                telemetry.addData("At Side", stateAt);
                telemetry.addData("At Mid", stateAtMid);
                telemetry.addData("Target Side", inchesSide);
                telemetry.addData("Target Middle", inchesMiddle);
                telemetry.addData("Current Side" , leftMotor.getCurrentPosition()/COUNTS_PER_INCH);
                telemetry.addData("Current Middle", middleMotor.getCurrentPosition()/COUNTS_PER_INCH_SIDE);
                if(Math.abs(leftMotor.getCurrentPosition()) < newSideTargets) {
                    telemetry.addLine("1");
                }
                if(Math.abs(rightMotor.getCurrentPosition()) < newSideTargets) {
                    telemetry.addLine("2");
                }
                if(Math.abs(middleMotor.getCurrentPosition()) < newMiddleTargets) {
                    telemetry.addLine("3");
                }
                if(Math.abs(middleMotor2.getCurrentPosition()) < newMiddleTargets) {
                    telemetry.addLine("4");
                }
                telemetry.update();
                runThroughs++;
            }
        }
        if(end == true) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor.setPower(0);
            middleMotor2.setPower(0);
        }

        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        telemetry.addData("Side Error", inchesSide - (leftMotor.getCurrentPosition()/COUNTS_PER_INCH));
        telemetry.addData("Middle Error", inchesMiddle - (middleMotor.getCurrentPosition()/COUNTS_PER_INCH_SIDE));
        telemetry.addData("Angle Error", startingAngle - Double.parseDouble(angleDouble));
        telemetry.update();

    }
    public void encoderDriveMiddle(double speed,//int leftDistance, int rightDistance,
                                   double middleInches,
                                   double timeoutS) throws InterruptedException {

        int newMiddleTarget = 0;
        //middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            newMiddleTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((middleInches) * COUNTS_PER_INCH);
            middleMotor.setTargetPosition(newMiddleTarget);
            middleMotor2.setTargetPosition(newMiddleTarget);
            // Turn On RUN_TO_POSITION
            middleMotor.setMode(RUN_TO_POSITION);
            middleMotor2.setMode(RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            middleMotor.setPower(speed);
            middleMotor2.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (middleMotor.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d", newMiddleTarget);
                //telemetry.addData("Path2", "Running at %7d",
                //        middleMotor.getCurrentPosition());
                //telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            middleMotor.setPower(0);
            middleMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDrive(double speed,//int leftDistance, int rightDistance,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {



        int newLeftTarget = 0;
        int newRightTarget = 0;
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int)((leftInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.rightMotor.getCurrentPosition() + */(int)((rightInches) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftMotor.setMode(RUN_TO_POSITION);
            rightMotor.setMode(RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Encoder Left", leftMotor.getCurrentPosition());
                telemetry.addData("Encoder Right", rightMotor.getCurrentPosition());
                //leftMotor.getCurrentPosition(),
                //rightMotor.getCurrentPosition();

                 telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveMotionProfileing(double speed,//int leftDistance, int rightDistance,
                                             double leftInches, double rightInches,
                                             double timeoutS) throws InterruptedException {



        int newLeftTarget = 0;
        int newRightTarget = 0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int)((leftInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.rightMotor.getCurrentPosition() + */(int)((rightInches) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftMotor.setMode(RUN_TO_POSITION);
            rightMotor.setMode(RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            int speedWeWant = 0;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy() )) {
                if(leftMotor.getCurrentPosition() == (newLeftTarget/3)*2) {

                }
                // Display it for the driver.
                telemetry.addData("Encoder Left", leftMotor.getCurrentPosition());
                telemetry.addData("Encoder Right", rightMotor.getCurrentPosition());
                //leftMotor.getCurrentPosition(),
                //rightMotor.getCurrentPosition();

                // telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveBoth(double speed, double speedMiddle,//int leftDistance, int rightDistance,
                                 double middleInches, double leftInches, double rightInches,
                                 double timeoutS) throws InterruptedException {

        int newMiddleTarget = 0;
        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            newMiddleTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((middleInches) * COUNTS_PER_INCH_SIDE);
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((rightInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((leftInches) * COUNTS_PER_INCH);
            middleMotor.setTargetPosition(newMiddleTarget);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            middleMotor2.setTargetPosition(newMiddleTarget);
            // Turn On RUN_TO_POSITION
            middleMotor.setMode(RUN_TO_POSITION);
            leftMotor.setMode(RUN_TO_POSITION);
            rightMotor.setMode(RUN_TO_POSITION);
            middleMotor2.setMode(RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            middleMotor.setPower(speedMiddle);
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            middleMotor2.setPower(speedMiddle);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (middleMotor.isBusy() || leftMotor.isBusy() || rightMotor.isBusy() || middleMotor2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Left Path", newLeftTarget);
                telemetry.addData("Middle Path", newMiddleTarget);
                telemetry.addData("Current Left", leftMotor.getCurrentPosition());
                telemetry.addData("Current Middle", middleMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            middleMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveAngle(double power, double inches, double angle) {
        int newMiddleTarget = 0;
        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            newMiddleTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((inches) * COUNTS_PER_INCH_SIDE * roundDouble(Math.sin(Math.toRadians(angle))));
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((inches) * COUNTS_PER_INCH * roundDouble(Math.cos(Math.toRadians(angle))));
            newRightTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((inches) * COUNTS_PER_INCH * roundDouble(Math.cos(Math.toRadians(angle))));
            middleMotor.setTargetPosition(newMiddleTarget);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            middleMotor2.setTargetPosition(newMiddleTarget);
            // Turn On RUN_TO_POSITION
            middleMotor.setMode(RUN_TO_POSITION);
            leftMotor.setMode(RUN_TO_POSITION);
            rightMotor.setMode(RUN_TO_POSITION);
            middleMotor2.setMode(RUN_TO_POSITION);
            pidStuff.f = 40;
            middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
            middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);

            // reset the timeout time and start motion.
            double midPower = roundDouble(power * Math.sin(Math.toRadians(angle)));
            double sidePower = roundDouble(power * Math.cos(Math.toRadians(angle)));
            middleMotor.setPower(midPower);
            leftMotor.setPower(sidePower);
            rightMotor.setPower(sidePower);
            middleMotor2.setPower(midPower);



            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    ((Math.abs(middleMotor.getCurrentPosition())) < Math.abs(roundDouble(Math.sin(Math.toRadians(angle))) * COUNTS_PER_INCH_SIDE * inches) || Math.abs(leftMotor.getCurrentPosition()) < (Math.abs(roundDouble(Math.cos(Math.toRadians(angle))) * COUNTS_PER_INCH * inches)) || Math.abs(rightMotor.getCurrentPosition()) < Math.abs(roundDouble(Math.cos(Math.toRadians(angle))) * COUNTS_PER_INCH * inches))) {

                // Display it for the driver.
                telemetry.addData("Left Path", newLeftTarget);
                telemetry.addData("Middle Path", newMiddleTarget);
                telemetry.addData("Current Left", leftMotor.getCurrentPosition());
                telemetry.addData("Current Middle", middleMotor.getCurrentPosition());
                telemetry.addData("Side Power", sidePower);
                telemetry.addData("Mid Power", midPower);
                if((Math.abs(middleMotor.getCurrentPosition())) < Math.abs(roundDouble(Math.sin(Math.toRadians(angle))) * COUNTS_PER_INCH_SIDE * inches)) {
                    telemetry.addLine("1");
                    telemetry.addData("Yea", roundDouble(Math.sin(Math.toRadians(angle))));
                }
                if(Math.abs(leftMotor.getCurrentPosition()) < (Math.abs(roundDouble(Math.cos(Math.toRadians(angle))) * COUNTS_PER_INCH * inches))) {
                    telemetry.addLine("2");
                }
                if(Math.abs(rightMotor.getCurrentPosition()) < Math.abs(roundDouble(Math.cos(Math.toRadians(angle))) * COUNTS_PER_INCH * inches)) {
                    telemetry.addLine("3");
                }
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            middleMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pidStuff.f = 11.2;
            middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
            middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);


            //  sleep(250);   // optional pause after each move
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
        while(leftMotor.isBusy() || rightMotor.isBusy() || middleMotor.isBusy() || middleMotor2.isBusy() || shoulderMotor.isBusy() || elbowMotor.isBusy() || rotationMotor.isBusy()) {
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
        hangArm.setPower(.7);
        Thread.sleep(100);
        //hangArm.setPower(0);

        hangArmLock.setPosition(.35);
        Thread.sleep(350);
        hangArm.setPower(0);

        //hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setMode(RUN_TO_POSITION);
        hangArm.setTargetPosition(-5600);
        hangArm.setPower(-.7);
        while(hangArm.isBusy()) {
            telemetry.addData("Pos", hangArm.getCurrentPosition());
            telemetry.update();
        }
        hangArm.setPower(0);


        middleMotor.setMode(RUN_TO_POSITION);
        middleMotor2.setMode(RUN_TO_POSITION);
        rotationMotor.setMode(RUN_TO_POSITION);
       //
         middleMotor.setTargetPosition(400);
        middleMotor2.setTargetPosition(400);
        rotationMotor.setTargetPosition(-1600);
        middleMotor.setPower(.6);
        middleMotor2.setPower(.6);
        rotationMotor.setPower(-.7);
        while(middleMotor.isBusy() || middleMotor2.isBusy() || rotationMotor.isBusy()) {
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
        while(Double.parseDouble(angleDouble) > (initialAngle + 1) || Double.parseDouble(angleDouble) < (initialAngle - 1)) {
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
        while(elbowMotor.isBusy() || shoulderMotor.isBusy() || rotationMotor.isBusy()) {
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
    public void adjustRotationMotor() {
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setTargetPosition(-1680);
        rotationMotor.setPower(-.7);
        while(rotationMotor.isBusy()) {

        }
        rotationMotor.setPower(0);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

}
