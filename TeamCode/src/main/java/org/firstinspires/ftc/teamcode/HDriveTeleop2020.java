package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@TeleOp(name = "SkyStone TeleOp", group = "HDrive")
public class HDriveTeleop2020 extends LinearOpMode {
    HDriveFCCalc calculator;
    HDriveFCCalc dpadCalculator;
    ArmCalculator armCalculator;
    ExtraClasses extraClasses;

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
    double xPos = 0;
    double yPos = 0;
    double leftServoPos = .5;
    double rightServoPos = .5;
    double startingAngle = 0;
    double rangeValuePrior = 50;
    double rangeSensorDistanceMid = 50;
    double filtered = 0;
    double loops = 0;

    boolean fieldCentric = true;
    boolean isPressedX = false;
    boolean armMode = false;
    boolean sideMoved = false;
    boolean dpadWasPressed = false;
    boolean updatedDpadMovement = false;
    boolean rightStickMoved = false;
    boolean newYPressed2 = true;
    boolean leftTriggerState = false;
    boolean leftTriggerPressed = false;
    boolean autoScoringMode = false;
    boolean aWasPressed2 = false;
    boolean driverHasControl = true;
    boolean autoScoringModeFirstTime = true;
    boolean brakeMode = false;
    boolean rightTriggerWasPressed = false;
    boolean isFieldCentricButton = false;
    boolean filteredValues = false;

    float rightX;
    long rightStickTimer = 0;
    boolean newAPressed2 = true;
    int armPos = 0;
    double time;
    int toggleVal = 0;
    boolean newToggle = true;



    //Robot Hardware
    DcMotorEx leftMotor;
    //DcMotorEx leftMotor2;
    DcMotorEx rightMotor;
    //DcMotorEx rightMotor2;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;
    DcMotorEx rightIntakeMotor;
    DcMotorEx leftIntakeMotor;
    Servo rightIntakeServo;
    Servo leftIntakeServo;
    Servo clawServo;
    DcMotorEx armFlipper;
    WebcamName webcamName = null;

    DistanceSensor rangeSensorLeft;
    DistanceSensor rangeSensorFront;
    DistanceSensor rangeSensorBack;

    //Vuforia Stuff
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY =
            "AWTzyDD/////AAABmXc9YGYk9kC9krb6KdWZ9hVC7/8o9Z/ZcGqEsCKx4HeT+G32mHKxDwKa4Hen6V52ikzlRbrqGXZuhKZPL8psMmuEoGlZ5yvHZLZjn/QcnFhc0D6g4UrXRKgYPYdXacdIgu/+KV1Tv1pNqY3vi4hyikc7ecWKTH2DVxL3CVqmtSjvJ1phlRJLaEx3GZW1YZ/IWvbA3+EDBSi4Kxbh5C84PWP5Ta1BcbPVn29PFNh9JlxIBkQdLCkkG2yp9RdKoOJFSG8IaYiz72FCaOa8UPTcxE+XoIgwfdLQwfQfmuJvx1E3h8iySlXl0UJjQZ2f03PbhxMuidv6vsTT/yTO5JvgOmPdIxlfCllRMKI22YCkRPz+";
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;           // the height of the center of the target image above the floor
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
    public List<VuforiaTrackable> allTrackables;
    VectorF translation;
    Orientation rotation;


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
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        //leftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor2");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");
        //rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor2");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor2");
        //rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        //leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        //clawServo = hardwareMap.get(Servo.class, "clawServo");
        //armFlipper = (DcMotorEx) hardwareMap.get(DcMotor.class, "armFlipper");
        rangeSensorFront = hardwareMap.get(DistanceSensor.class, "Range Sensor Front");
        rangeSensorBack = hardwareMap.get(DistanceSensor.class, "Range Sensor Back");
        rangeSensorLeft = hardwareMap.get(DistanceSensor.class, "Range Sensor Left");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        calculator = new HDriveFCCalc();
        dpadCalculator = new HDriveFCCalc();
        armCalculator = new ArmCalculator(bicep, forearm);
        extraClasses = new ExtraClasses(leftMotor, rightMotor, middleMotor, middleMotor2);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        //leftMotor2.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor2.setDirection(DcMotor.Direction.REVERSE);

        /*leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle));
        holdAngle = angleDouble;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 10;
        pidStuff.i = 2;
        pidStuff.d = 0;
        pidStuff.f = 14;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        //leftMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        //rightMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);

        timeDifferenceBetweenLoops = System.currentTimeMillis();

        //More Vuforia Stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        allTrackables = new ArrayList<VuforiaTrackable>();
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
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, vuforiaParameters.cameraDirection);
        }
        targetsSkyStone.activate();

        telemetry.addLine("Ready to Begin");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = Double.parseDouble(ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle));
            gamepadMovements();
            checkFieldCentricAndSlowMode();
            moveTheBase();
            runIntake();
            checkEncoderModes();
            tankDriveOdometry();
            autoScoreMode();
            runVuforia();
            moveArm();
            controlIntake();
            averageLoops();
            /*telemetry.addData("Left Move", calculator.getLeftDrive());
            telemetry.addData("Right Move", calculator.getRightDrive());
            telemetry.addData("Middle Move", calculator.getMiddleDrive());
            telemetry.addData("Angle", angleDouble);
            telemetry.addData("Adjusted Angle", extraClasses.convertAngle(angleDouble));
            telemetry.addData("Middle Power", middleMotor.getPower());
            telemetry.addData("Middle 2 Power", middleMotor2.getPower());
            telemetry.addData("Auto Scoring Mode", autoScoringMode);
            telemetry.update();*/
        }
    }

    public void gamepadMovements() {

        double yJoystick = 0;
        double xJoystick = 0;
        if (gamepad2.dpad_up) {

        }
        if (gamepad2.dpad_down) {

        }
        if (gamepad1.dpad_up) {
            dpadWasPressed = true;
            yJoystick = -.3;
        } else if (gamepad1.dpad_down) {
            dpadWasPressed = true;
            yJoystick = .3;
        } else {
            yJoystick = 0;
        }
        if (gamepad1.dpad_left) {
            dpadWasPressed = true;
            xJoystick = -.3;
        } else if (gamepad1.dpad_right) {
            dpadWasPressed = true;
            xJoystick = .3;
        } else {
            xJoystick = 0;

        }
        if(driverHasControl) {
            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                updatedDpadMovement = true;
                dpadCalculator.calculateMovement(xJoystick, yJoystick, 0, angleDouble + offset);
                middleMotor.setPower(dpadCalculator.getMiddleDrive());
                middleMotor2.setPower(dpadCalculator.getMiddleDrive());
                leftMotor.setPower(dpadCalculator.getLeftDrive());
                //leftMotor2.setPower(dpadCalculator.getLeftDrive());
                rightMotor.setPower(dpadCalculator.getRightDrive());
                //rightMotor2.setPower(dpadCalculator.getRightDrive());
            } else if ((leftMotor.getPower() != 0 || rightMotor.getPower() != 0 || middleMotor.getPower() != 0) && updatedDpadMovement) {
                leftMotor.setPower(0);
                //leftMotor2.setPower(0);
                rightMotor.setPower(0);
                //rightMotor2.setPower(0);
                middleMotor.setPower(0);
                middleMotor2.setPower(0);
                updatedDpadMovement = false;
            }
        }

        if (gamepad1.right_trigger > .5 && !rightTriggerWasPressed) {
            rightTriggerWasPressed = true;
            brakeMode = !brakeMode;
            if (brakeMode) {
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                middleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                middleMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                middleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                middleMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        } else if (gamepad1.right_trigger < .5) {
            rightTriggerWasPressed = false;
        }
    }

    public void checkFieldCentricAndSlowMode() {
        if (gamepad2.x && isPressedX) {
            isPressedX = false;
            if (armMode) {
                armMode = false;
            } else {
                armMode = true;
            }
        } else if (!gamepad2.x) {
            isPressedX = true;
        }
    }

    public void moveTheBase() {
        if (!gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_down && !gamepad1.dpad_up) {
            leftX = gamepad1.left_stick_x;
            leftY = gamepad1.left_stick_y;
            rightX = gamepad1.right_stick_x;
            if (rightX != 0) {
                holdAngle = angleDouble;
                rightStickMoved = true;
                rightStickTimer = System.currentTimeMillis();
            } else if (rightStickMoved && rightX == 0) {
                double timeDifference = System.currentTimeMillis() - rightStickTimer;
                holdAngle = angleDouble;
                if (timeDifference > 500) {
                    rightStickMoved = false;
                }
            }
            if (gamepad1.b == true) {
                offset = angleDouble + 180;
                offset = -offset;
            }

            if (gamepad1.x && isFieldCentricButton) {
                isFieldCentricButton = false;
                fieldCentric = !fieldCentric;
            } else if (!gamepad1.x) {
                isFieldCentricButton = true;
            }

            if (fieldCentric) {
                calculator.calculateMovement(leftX, leftY, rightX, angleDouble + offset);
            } else {
                calculator.calculateMovement(-leftX, -leftY, rightX, 0);
            }
            if (calculator.getLeftDrive() != 0 || calculator.getRightDrive() != 0 || calculator.getMiddleDrive() != 0) {
                sideMoved = true;
            }
            maintainAngle();
            if (driverHasControl) {
                leftMotor.setPower(speed * calculator.getLeftDrive() /*+ sideChangePower*/);
                //leftMotor2.setPower(speed * calculator.getLeftDrive() /*+ sideChangePower*/);
                rightMotor.setPower(speed * calculator.getRightDrive() /*- sideChangePower*/);
                //rightMotor2.setPower(speed * calculator.getRightDrive() /*- sideChangePower*/);
                middleMotor.setPower(speed * calculator.getMiddleDrive() * 2);
                middleMotor2.setPower(speed * calculator.getMiddleDrive() * 2);
            }
            driverHasControl = true;
        }
    }

    public void tankDriveOdometry() {
        here++;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(extraClasses.formatAngle(angles.angleUnit, angles.firstAngle));
        double leftPosChange = leftMotor.getCurrentPosition() - lastLeftPos;
        double rightPosChange = rightMotor.getCurrentPosition() - lastRightPos;
        lastLeftPos = leftMotor.getCurrentPosition();
        lastRightPos = rightMotor.getCurrentPosition();
        xPos = (xPos + ((leftPosChange + rightPosChange) / 2) * Math.sin(Math.toRadians(extraClasses.convertAngle(angleDouble))));
        yPos = (yPos + ((leftPosChange + rightPosChange) / 2) * Math.cos(Math.toRadians(extraClasses.convertAngle(angleDouble))));
    }

    public void maintainAngle() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle));
        sideChangePower = (extraClasses.convertAngle(angleDouble) - extraClasses.convertAngle(holdAngle)) / 75;
    }

    public void checkEncoderModes() {
        if ((!leftMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) || !rightMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) || !middleMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER))) {
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void runIntake() {
        //Set Trigger to Toggle and Check if its pressed
        if (gamepad2.left_trigger > .5 && leftTriggerPressed == false) {
            leftTriggerPressed = true;
            if (leftTriggerState) {
                leftTriggerState = false;
            } else {
                leftTriggerState = true;
            }
        } else {
            leftTriggerPressed = false;
        }
        //Delete this Later
        if (gamepad2.left_bumper && leftServoPos < 1) {
            leftServoPos = leftServoPos + .04;
        } else if (gamepad2.left_trigger == 1 && leftServoPos > 0) {
            leftServoPos = leftServoPos - .04;
        }
        if (gamepad2.right_bumper && rightServoPos > 0) {
            rightServoPos = rightServoPos - .04;
        } else if (gamepad2.right_trigger == 1 && rightServoPos < 1) {
            rightServoPos = rightServoPos + .04;
        }
        leftIntakeServo.setPosition(leftServoPos);
        rightIntakeServo.setPosition(rightServoPos);
        //Set the intake servos to the two predetermined postions
        /*if(gamepad2.left_bumper && leftBumperPressed == false) {
            leftBumperPressed = true;
            if(intakeState == 0) {
                intakeState = 1;
                leftIntakeServo.setPosition(.5);
                rightIntakeServo.setPosition(.5);
            } else if(intakeState == 1) {
                intakeState = 0;
                leftIntakeServo.setPosition(1);
                rightIntakeServo.setPosition(1);
            }
        }
        else {
            leftBumperPressed = false;
        }

        //Actually Run the Intake
        if(leftTriggerState) {
            double leftDistance = sensorRangeLeft.getDistance(DistanceUnit.CM);
            double rightDistance = sensorRangeRight.getDistance(DistanceUnit.CM);
            if((leftDistance - rightDistance > 4)) {
                //leftIntakeMotor.setPower(-.4);
                //rightIntakeMotor.setPower(.2);
            }
            else if((leftDistance - rightDistance) < -4) {
                //leftIntakeMotor.setPower(.2);
                //rightIntakeMotor.setPower(-.4);
            }
            else if(Math.abs((leftDistance - rightDistance)) < 4) {
                //leftIntakeMotor.setPower(-.4);
                //rightIntakeMotor.setPower(-.4);
            }
        }
        else {
            //leftIntakeMotor.setPower(0);
            //rightIntakeMotor.setPower(0);
        }*/

    }

    public void runVuforia() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            translation = lastLocation.getTranslation();
            //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
            //translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
        }
        //telemetry.update();
    }

    public void moveArm() {
        //closes claw and moves arm all the way ove the robot
        /*if (gamepad2.y && newYPressed2) {
            newYPressed2 = false;
            switch (armPos) {
                case 0:
                    // currently home
                    clawServo.setPosition(.5);
                    time = System.currentTimeMillis();
                    armFlipper.setMode(RUN_TO_POSITION);
                    armFlipper.setTargetPosition(1);
                    break;
                case 1:
                    // currently extended
                    clawServo.setPosition(.28);
                    time = System.currentTimeMillis();
                    armFlipper.setMode(RUN_TO_POSITION);
                    armFlipper.setTargetPosition(0);
                    break;
                default:
                    telemetry.addData("Something went wrong and it is all darby's fault", 0);
            }

            armPos += 1;
            armPos = armPos % 2;
        } else if (!gamepad2.y) {
            newYPressed2 = true;
            if (armFlipper.isBusy() && System.currentTimeMillis() >= time + 1000) {
                switch (armPos) {
                    case 0:
                        // trying to go home
                        armFlipper.setPower(-1);
                        break;
                    case 1:
                        // trying to extend
                        armFlipper.setPower(1);
                        break;
                    default:
                        telemetry.addData("Something went wrong and it is all darby's fault", 1);
                }
            }
        } else if (armFlipper.isBusy() && System.currentTimeMillis() >= time + 1000) {
            switch (armPos) {
                case 0:
                    // trying to go home
                    armFlipper.setPower(-1);
                    break;
                case 1:
                    // trying to extend
                    armFlipper.setPower(1);
                    break;
                default:
                    telemetry.addData("Something went wrong and it is all darby's fault", 69);
            }


        }
        if (!armFlipper.isBusy() && gamepad2.right_stick_x != 0) {
            armFlipper.setPower(gamepad2.right_stick_x);
        }*/
        if (gamepad1.a && newAPressed2) {
            if (ExtraClasses.closeEnough(clawServo.getPosition(), /* open*/ .5, .05)) {
                clawServo.setPosition(/* closed*/.28);
            } else if (ExtraClasses.closeEnough(clawServo.getPosition(), /* closed*/ .28, .05)) {
                clawServo.setPosition(/* open*/.5);
            } else {
                telemetry.addData("Something went wrong and it is all darby's fault - opening servo", 2);
                clawServo.setPosition(/* open*/.5);
            }
        } else if (!gamepad1.a) {
            newAPressed2 = false;
        }

    }

    public void autoScoreMode() {
        if (gamepad1.a && aWasPressed2 == false) {
            aWasPressed2 = true;
            if (autoScoringMode == false) {
                autoScoringMode = true;
            } else {
                autoScoringMode = false;
            }

        } else if (gamepad1.a != true) {
            aWasPressed2 = false;
        }

        //Driver does not have control of robot here
        if (autoScoringMode) {
            if (autoScoringModeFirstTime) {
                autoScoringModeFirstTime = false;
                startingAngle = extraClasses.convertAngle(angleDouble);
            }
            driverHasControl = false;

            //Find Angle Error
            double goalAngle = 90; //Just the starting angle I think
            double distance1 = Math.abs(angleDouble - goalAngle);
            double distance2 = Math.abs(Math.abs((360 - angleDouble)) - goalAngle);
            double angleError = distance1;
            if(distance1 > distance2) {
                angleError = distance2;
            }
            if((goalAngle - angleDouble + 360) % 360 < 180) {
                angleError = angleError * -1;
            }
            else {
                angleError = angleError;
            }
            angleError = angleError / 60;

            //Find how far from the side wall it is
            double setDistanceItShoudldBeBack = 18;
            double rangeSensorDistanceBack = rangeSensorBack.getDistance(DistanceUnit.CM);
            double distanceDifferenceBack = rangeSensorDistanceBack - setDistanceItShoudldBeBack;
            double frontPowerError = distanceDifferenceBack / 50;

            //Find how far from the foundation it is
            double setDistanceItShoudldBeMid = 50;
            if(filteredValues) {
                rangeValuePrior = rangeValuePrior;
            } else {
                rangeValuePrior = rangeSensorDistanceMid;
            }
            rangeSensorDistanceMid = rangeSensorLeft.getDistance(DistanceUnit.CM);
            double rangeSensorValueUsed = filterValues(rangeSensorDistanceMid, rangeValuePrior,10);
            double distanceDifferenceMid = rangeSensorValueUsed - setDistanceItShoudldBeMid;
            double middlePowerError = distanceDifferenceMid / 40;

            leftMotor.setPower(-frontPowerError + angleError);
            rightMotor.setPower(-frontPowerError + -angleError);
            middleMotor2.setPower(middlePowerError);

            telemetry.addData("Angle Error", angleError);
            telemetry.addData("Front Error", distanceDifferenceBack);
            telemetry.addData("Front Distance", rangeSensorBack.getDistance(DistanceUnit.CM));
            telemetry.addData("Mid Distance", rangeSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Filtered Values", filteredValues);
            telemetry.addData("Filtered", filtered);
            telemetry.addData("Prior Mid", rangeValuePrior);
            telemetry.update();

        } else {
            autoScoringModeFirstTime = true;
        }

    }
    public void controlIntake(){
        boolean angleToggle = gamepad2.right_bumper;
        double motorVal = gamepad2.right_trigger - gamepad2.left_trigger;

        if (angleToggle && newToggle){
            if (toggleVal == 0){
                leftIntakeServo.setPosition(.5);
                rightIntakeServo.setPosition(.5);
            } else if (toggleVal == 1){
                leftIntakeServo.setPosition(0);
                rightIntakeServo.setPosition(0);
            }
            toggleVal += 1;
            toggleVal = toggleVal % 2;
        }
        if (!angleToggle){
            newToggle = true;
        }
        //rightIntakeMotor.setPower(motorVal);
        //leftIntakeMotor.setPower(-motorVal);
    }

    public double filterValues(double currentValue, double previousValue, double tolerance) {
        if(Math.abs(currentValue - previousValue) < tolerance) {
            filtered = 0;
            filteredValues = false;
            return currentValue;
        } else {
            filteredValues = true;
            filtered++;
            if(filtered == 1) {
                telemetry.addData("error", currentValue);
                telemetry.addData("filtered times", filtered);
                telemetry.update();
            }
            return previousValue;
        }
    }
    public void averageLoops (){
        loops ++;
        telemetry.addData("Totals Loops", loops);
        telemetry.addData("Average loops per second:", 1000*(loops/System.currentTimeMillis()));
    }
}
