package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

@TeleOp(name = "Fresh Boi", group = "HDrive")
public class MrFresh extends OpMode {
    HDriveFCCalc calculator;
    HDriveFCCalc dpadCalculator;
    ArmCalculator armCalculator;
    ExcessStuff excessStuff;

    //Ints, Doubles, Booleans, and Floats
    int landerState = 0;
    int hangArmLockCounter = 0;
    int priorRotationPos = 2982;
    int priorElbowPos = -1113;
    int priorShoulderPos = 3400;
    int here = 0;
    int sideAdjusted = 422422;
    int middleAdjusted = 422422;
    double offset = 270;
    double bicep = 18;
    double forearm = 17.25;
    double speed = 1;
    double startingAngle = 0;
    double startingAngleLander = 0;
    double endingAngle = 0;
    double angleError = 0;
    double sideChange = 0;
    double sidePower = 0;
    double offsetSide = 0;
    double offsetMid = 0;
    double craterState = 0;
    double powerMultiplier = 0;
    double rotationPos = 0;
    double sideChangePower = 0;
    double leftTime = System.currentTimeMillis();
    double rightTime = System.currentTimeMillis();

    boolean firstTimeStopedLander = true;
    boolean fieldCentric = true;
    boolean yPressed = false;
    boolean isGettingOnLander = false;
    boolean firstTimeLander = true;
    boolean zeroPowerBehavior = false;
    boolean isPressedX = false;
    boolean isPressedY1Gamepad = false;
    boolean isFieldCentricButton = false;
    boolean armMode = false;
    boolean isRunningArm = false;
    boolean armMoved = false;
    boolean sideMoved = false;
    boolean heldPositionLast = false;
    boolean firstTimeMovingLander = true;
    boolean isMovingTowardsLander = false;
    boolean nowTurning = false;
    boolean moveToBlocks = false;
    boolean ballInLeft = false;
    boolean ballInRight = false;
    boolean bumperPressedLander = false;
    boolean firstTimeNotSeeingB = true;
    boolean dpadWasPressed = false;
    boolean firstTimeGeneral = true;
    boolean updatedDpadMovement = false;
    boolean buttonAToggle = false;
    boolean buttonAWasPressed = false;
    boolean reachedGoal = false;
    boolean leftIntakeWasUsed = false;
    boolean rightIntakeWasUsed = false;
    boolean leftIntakeFirst = true;
    boolean rightIntakeFirst = true;
    int heldElbowPos = 0;
    int heldShoulderPos = 0;
    int hangerState = 0;
    float rightX;
    double leftX;
    double leftY;
    double holdAngle = 0;
    double timeDifferenceBetweenLoops = System.currentTimeMillis();
    String angleDouble = "hi";

    //Robot Hardware
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;
    DcMotorEx hangArm;

    DcMotorEx shoulderMotor;
    DcMotorEx elbowMotor;
    DcMotorEx rotationMotor;
    Servo hangArmLock;
    Servo leftIntake;
    Servo rightIntake;
    Servo bouncer;

    Orientation angles;
    BNO055IMU imu;
    PIDFCoefficients pidStuff;

    public void init() {
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
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor2");
        shoulderMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Shoulder Motor");
        elbowMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Elbow Motor");
        rotationMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Rotation Motor");
        leftIntake = hardwareMap.get(Servo.class, "Left Intake");
        rightIntake = hardwareMap.get(Servo.class, "Right Intake");
        bouncer = hardwareMap.get(Servo.class, "Bouncer");
        hangArm = (DcMotorEx) hardwareMap.get(DcMotor.class, "Hang Arm");
        hangArmLock = hardwareMap.servo.get("Hang Arm Lock");
        calculator = new HDriveFCCalc();
        dpadCalculator = new HDriveFCCalc();
        armCalculator = new ArmCalculator(bicep, forearm);
        excessStuff = new ExcessStuff(leftMotor, rightMotor, middleMotor, middleMotor2, elbowMotor, shoulderMotor, rotationMotor);
        elbowMotor.setDirection(DcMotor.Direction.REVERSE);
        rotationMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
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
        angleDouble = ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle);
        holdAngle = Double.parseDouble(angleDouble);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 5;
        pidStuff.i = 4;
        pidStuff.d = 0;
        pidStuff.f = 10.2;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightIntake.setDirection(Servo.Direction.REVERSE);
        leftIntake.setPosition(.5);
        rightIntake.setPosition(.5);
        bouncer.setPosition(0);
        hangArmLock.setPosition(.35);

        rotationPos = rotationMotor.getCurrentPosition();
        timeDifferenceBetweenLoops = System.currentTimeMillis();
    }

    public void loop() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle);
        here++;
        armCalculator.calculateSpeed(gamepad2.left_stick_x, gamepad2.right_stick_y, shoulderMotor, elbowMotor, ExcessStuff.shoulderAngle((double) shoulderMotor.getCurrentPosition()), ExcessStuff.elbowAngle((double) elbowMotor.getCurrentPosition()), telemetry);
        telemetry.addData("Hold Angle", ExcessStuff.convertAngle(holdAngle));
        telemetry.addData("Current Angle", ExcessStuff.convertAngle(Double.parseDouble(angleDouble)));
        //telemetry.addData("left intake", leftIntakeWasUsed);
        //telemetry.addData("Thing",((System.currentTimeMillis() - leftTime)));
        telemetry.update();

        dpadCheck();
        checkArmMovement();
        checkFieldCentricAndSlowMode();
        checkIntake();
        gettingOnTheLander();
        moveTheBase();
        ScoringAndCollectingButtons();
        //setZeros();
        checkAutoArmButtons();
        checkDeadReckonButton();
        checkEncoderModes();

        if (!armMoved) {
            holdArmPosition();
        } else {
            heldPositionLast = false;
        }
        /*telemetry.addData("Side Moved", sideMoved);
        if(!sideMoved) {
            holdSideMotorsPosition();
        }
        else {
            heldSidePositionLast = false;
        }*/
        armMoved = false;
        sideMoved = false;
    }

    public void dpadCheck() {
        double sidePower = 0;
        double midPower = 0;
        double yJoystick = 0;
        double xJoystick = 0;
        if (gamepad2.dpad_up) {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(50);
            elbowMotor.setTargetPosition(300);
            rotationMotor.setTargetPosition(-25);
            shoulderMotor.setPower(.5);
            elbowMotor.setPower(0.5);
            rotationMotor.setPower(.3);
            /*(telemetry.addData("elbow pos", elbowMotor.getCurrentPosition());
            telemetry.addData("shoulder pos", shoulderMotor.getCurrentPosition());
            telemetry.addData("rotation Pos", rotationMotor.getCurrentPosition());
            telemetry.update();*/
            armMoved = true;
            isRunningArm = true;
        }
        if (gamepad2.dpad_down) {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(3982);
            elbowMotor.setTargetPosition(-118);
            shoulderMotor.setPower(.7);
            elbowMotor.setPower(0.7);
            /*telemetry.addData("elbow pos", elbowMotor.getCurrentPosition());
            telemetry.addData("shoulder pos", shoulderMotor.getCurrentPosition());
            telemetry.update();*/
            armMoved = true;
            isRunningArm = true;
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
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            updatedDpadMovement = true;
            dpadCalculator.calculateMovement(xJoystick, yJoystick, 0, Double.parseDouble(angleDouble) + offset);
            middleMotor.setPower(dpadCalculator.getMiddleDrive());
            middleMotor2.setPower(dpadCalculator.getMiddleDrive());
            leftMotor.setPower(dpadCalculator.getLeftDrive());
            rightMotor.setPower(dpadCalculator.getRightDrive());
        } else if ((leftMotor.getPower() != 0 || rightMotor.getPower() != 0 || middleMotor.getPower() != 0) && updatedDpadMovement) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor.setPower(0);
            middleMotor2.setPower(0);
            updatedDpadMovement = false;
        }

        /*telemetry.addData("dpadCalc", dpadCalculator.getLeftDrive());
        telemetry.addData("dpadCalc", dpadCalculator.getMiddleDrive());
        telemetry.addData("X Joystick", xJoystick);
        telemetry.addData("Y Joystick", yJoystick);
        telemetry.addData("Left mode", leftMotor.getMode());
        telemetry.addData("Mid Mode", middleMotor.getMode());
        telemetry.update();*/
    }

    public void checkArmMovement() {
        if (armMode) {
            if ((gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0) && !buttonAToggle) {
                zeroPowerBehavior = false;
                armMoved = true;
                releaseArm();
                if (rotationMotor.getCurrentPosition() < 3307 && rotationMotor.getCurrentPosition() > 25) {
                    rotationMotor.setPower(.6);
                } else if (rotationMotor.getCurrentPosition() >= 3307 && (.5 * gamepad2.right_stick_x) < 0) {
                    rotationMotor.setPower(.6);
                } else if (rotationMotor.getCurrentPosition() <= 25 && (.5 * gamepad2.right_stick_x) > 0) {
                    rotationMotor.setPower(.6);
                } else {
                    rotationMotor.setPower(.3);
                }
                elbowMotor.setPower(.5 * -gamepad2.left_stick_y);
                shoulderMotor.setPower(.5 * gamepad2.left_stick_x);
            }
        } else if (armMode == false) {
            if (gamepad2.x && isPressedX) {
                isPressedX = false;
                if (armMode) {
                    armMode = false;
                } else {
                    //armCalculator.setTargetYToCurrent(ExcessStuff.shoulderAngle(shoulderMotor.getCurrentPosition()),ExcessStuff.elbowAngle(elbowMotor.getCurrentPosition()));
                    armMode = true;
                }
            } else if (!gamepad2.x) {
                isPressedX = true;
            }
            if (gamepad2.b && (gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0)) {
                releaseArm();
                rotationMotor.setPower(.3 * gamepad2.right_stick_x);
                elbowMotor.setPower(.2 * armCalculator.getElbowMotorSpeed());
                shoulderMotor.setPower(.2 * armCalculator.getShoulderMotorSpeed());
                zeroPowerBehavior = false;
                armMoved = true;
            } else if ((gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0) && !buttonAToggle) {
                releaseArm();
                if (rotationMotor.getCurrentPosition() < 3307 && rotationMotor.getCurrentPosition() > 25) {
                    rotationMotor.setPower(.6);
                } else if (rotationMotor.getCurrentPosition() >= 3307 && (gamepad2.right_stick_x) < 0) {
                    rotationMotor.setPower(.6);
                } else if (rotationMotor.getCurrentPosition() <= 25 && (.5 * gamepad2.right_stick_x) > 0) {
                    rotationMotor.setPower(.6);
                } else {
                    rotationMotor.setPower(0);
                }
                elbowMotor.setPower(.4 * armCalculator.getElbowMotorSpeed());
                shoulderMotor.setPower(.4 * armCalculator.getShoulderMotorSpeed());
                zeroPowerBehavior = false;
                armMoved = true;
            }
        }
        if (gamepad2.dpad_right) {
            armCalculator.setTargetY(0);
            armMoved = true;
        }
        timeDifferenceBetweenLoops = System.currentTimeMillis() - timeDifferenceBetweenLoops;
        if (gamepad2.right_stick_x != 0) {
            if (!ExcessStuff.closeEnough(rotationMotor.getCurrentPosition(), rotationPos, 10)) {
                rotationPos = rotationMotor.getCurrentPosition();
            }
            rotationPos = rotationPos + (5 * gamepad2.right_stick_x * timeDifferenceBetweenLoops);
            rotationMotor.setTargetPosition((int) rotationPos);
        }
        timeDifferenceBetweenLoops = System.currentTimeMillis();
        if (gamepad2.y) {
            if (!elbowMotor.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            armMoved = true;
            elbowMotor.setPower(-.1);

        }
        if (rotationMotor.getCurrentPosition() != rotationPos && !buttonAToggle && !gamepad2.dpad_up && gamepad2.left_stick_x == 0 && gamepad2.left_stick_y == 0 && gamepad2.right_stick_x == 0) {
            if (!ExcessStuff.closeEnough(rotationMotor.getCurrentPosition(), rotationPos, 50)) {
                rotationPos = rotationMotor.getCurrentPosition();
            }
            rotationMotor.setTargetPosition((int) rotationPos);
            rotationMotor.setPower(.1);
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

    public void checkIntake() {
        if (!buttonAToggle) {
            if (gamepad2.left_trigger == 1) {
                leftIntake.setPosition(.05); //collects
            } else {
                if (gamepad2.left_bumper) {
                    leftIntake.setPosition(0.5 + (1 / 2.3)); //scores
                    leftIntakeWasUsed = true;
                    leftIntakeFirst = true;
                } else {
                    if (!leftIntakeWasUsed) {
                        leftIntake.setPosition(0.5 + (0 / 2.3)); //scores
                    }
                }
            }
            if (gamepad2.right_trigger == 1) {
                rightIntake.setPosition(.05); //collects
            } else {
                if (gamepad2.right_bumper) {
                    rightIntake.setPosition(0.5 + (1 / 2.3));//score
                    rightIntakeWasUsed = true;
                    rightIntakeFirst = true;
                } else {
                    if (!rightIntakeWasUsed) {
                        rightIntake.setPosition(0.5 + (0 / 2.3));//scores
                    }
                }
            }
        }
        if (leftIntakeWasUsed && !gamepad2.left_bumper) {
            if (leftIntakeFirst) {
                leftTime = System.currentTimeMillis();
                leftIntakeFirst = false;
            }
            if (((System.currentTimeMillis() - leftTime) < 1000)) {
                leftIntake.setPosition(.6);
            } else {
                leftIntakeWasUsed = false;
                leftIntakeFirst = true;
            }

        }
        if (rightIntakeWasUsed && !gamepad2.right_bumper) {
            if (rightIntakeFirst) {
                rightTime = System.currentTimeMillis();
                rightIntakeFirst = false;
            }
            if ((System.currentTimeMillis() - rightTime) < 1000) {
                rightIntake.setPosition(.6);
            } else {
                rightIntakeFirst = true;
                rightIntakeWasUsed = false;
            }
        }
    }

    public void gettingOnTheLander() {
        if (gamepad1.left_bumper) {
            getOnLander();
            isGettingOnLander = true;
        } else {
            hangerState = 0;
            isGettingOnLander = false;
            firstTimeLander = true;
        }
        if (gamepad1.right_trigger == 1 && !buttonAToggle) {
            goToLander();
            isMovingTowardsLander = true;
        } else {
            if (isMovingTowardsLander) {
                pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                pidStuff.p = 0;
                pidStuff.i = 0;
                pidStuff.d = 0;
                pidStuff.f = 0;
                pidStuff.algorithm = MotorControlAlgorithm.PIDF;
                leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                middleMotor.setPower(0);
                middleMotor2.setPower(0);
                pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                pidStuff.p = 5;
                pidStuff.i = 4;
                pidStuff.d = 0;
                pidStuff.f = 10.2;
                pidStuff.algorithm = MotorControlAlgorithm.PIDF;
                leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
            }
            isMovingTowardsLander = false;
            firstTimeMovingLander = true;
            firstTimeStopedLander = true;
        }
    }

    public void moveTheBase() {
        if (isGettingOnLander == false && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_down && !gamepad1.dpad_up && !isMovingTowardsLander && !buttonAToggle) {
            if (yPressed == false) {
                leftX = gamepad1.left_stick_x;
                leftY = gamepad1.left_stick_y;
            }
            if (buttonAToggle != true && yPressed == false) {
                rightX = gamepad1.right_stick_x;
                if(rightX != 0) {
                    holdAngle = Double.parseDouble(angleDouble);
                }
            }
            if (gamepad1.b == true) {
                offset = Double.parseDouble(angleDouble) + 180;
                offset = -offset;
            }

            if (gamepad1.x && isFieldCentricButton) {
                isFieldCentricButton = false;
                if (fieldCentric) {
                    fieldCentric = false;
                } else {
                    fieldCentric = true;
                }
            } else if (!gamepad1.x) {
                isFieldCentricButton = true;
            }

            if (fieldCentric) {
                calculator.calculateMovement(leftX, leftY, rightX, Double.parseDouble(angleDouble) + offset);
            } else {
                calculator.calculateMovement(-leftX, -leftY, rightX, 0);
            }
            if (gamepad1.left_stick_button && isPressedY1Gamepad) {
                if (speed == 1) {
                    speed = .5;
                } else {
                    speed = 1;
                }
                isPressedY1Gamepad = false;
            } else if (!gamepad1.left_stick_button) {
                isPressedY1Gamepad = true;
            }
            if (calculator.getLeftDrive() != 0 || calculator.getRightDrive() != 0 || calculator.getMiddleDrive() != 0) {
                sideMoved = true;
            }
            maintainAngle();
            leftMotor.setPower(speed * calculator.getLeftDrive());
            rightMotor.setPower(speed * calculator.getRightDrive());
            middleMotor.setPower(speed * calculator.getMiddleDrive());
            middleMotor2.setPower(speed * calculator.getMiddleDrive());
        }
    }

    public void holdArmPosition() {
        if (!heldPositionLast) {
            heldElbowPos = elbowMotor.getCurrentPosition();
            heldShoulderPos = shoulderMotor.getCurrentPosition();
            heldPositionLast = true;
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        elbowMotor.setPower(.2);
        shoulderMotor.setPower(.2);
        //rotationMotor.setPower(0);
        elbowMotor.setTargetPosition(heldElbowPos);
        shoulderMotor.setTargetPosition(heldShoulderPos);

    }

    public void releaseArm() {
        if (shoulderMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || elbowMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || rotationMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            ;
        {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void goToLander() {
        if (firstTimeMovingLander) {
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPosition(0 + (int) offsetSide); //-2000
            rightMotor.setTargetPosition(0 + (int) offsetSide);//-2000
            middleMotor.setTargetPosition(0 + (int) offsetMid);//-4000
            middleMotor2.setTargetPosition(0 + (int) offsetMid);//-4000
            leftMotor.setPower(.4);
            rightMotor.setPower(.4);
            middleMotor.setPower(.9);
            middleMotor2.setPower(.9);
            firstTimeMovingLander = false;
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle);
            if (leftMotor.getCurrentPosition() > 0 + (int) offsetSide) {
                sidePower = -.4;
            } else if (leftMotor.getCurrentPosition() < 0 + (int) offsetSide) {
                sidePower = .4;
            }
        }
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle);
        endingAngle = Double.parseDouble(angleDouble);
        angleError = endingAngle - startingAngle;
        sideChange = (angleError) / 100;
        if (leftMotor.isBusy() && rightMotor.isBusy()) {
            leftMotor.setPower(sidePower + sideChange);
            rightMotor.setPower(sidePower - sideChange);
        }
        if (!rightMotor.isBusy() || !leftMotor.isBusy()) {
            if (rightMotor.getMode() == RUN_TO_POSITION || leftMotor.getMode() == RUN_TO_POSITION) {
                leftMotor.setMode(RUN_USING_ENCODER);
                rightMotor.setMode(RUN_USING_ENCODER);
            }
            leftMotor.setPower(sideChange);
            rightMotor.setPower(-sideChange);
        }
        if (!leftMotor.isBusy() && !rightMotor.isBusy() && !middleMotor.isBusy() && !middleMotor2.isBusy()) {
            if (firstTimeStopedLander) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                middleMotor.setPower(0);
                middleMotor2.setPower(0);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                firstTimeStopedLander = false;
            }
        }
        sideMoved = true;
    }
    public void maintainAngle() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle);
        if (holdAngle < Double.parseDouble(angleDouble)) {
            sideChangePower = Math.abs(Double.parseDouble(angleDouble)) - Math.abs(holdAngle);
        }
    }
    public void getOnLander() {
        if (firstTimeLander) {
            hangerState = 0;
            hangArmLockCounter = 0;
        }
        firstTimeLander = false;

        if (hangerState == 0) {
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setPower(-.2);
            middleMotor2.setPower(-.2);
            hangerState = 1;
        } else if (hangerState == 1) {
            if (gamepad1.right_bumper == true) {
                middleMotor.setPower(0);
                middleMotor2.setPower(0);
                hangerState = 2;
            }
        } else if (hangerState == 2) {
            hangArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);//.522
            hangArm.setTargetPosition(-1000);
            hangArm.setPower(-.5);
            hangerState = 3;
        } else if (hangerState == 3) {
            if (ExcessStuff.closeEnough(hangArm.getCurrentPosition(), -1000, 25)) {
                hangArm.setPower(.4);
                hangArm.setTargetPosition(hangArm.getCurrentPosition());
                hangArmLock.setPosition(.534);
                hangerState = 4;
            }
        } else if (hangerState == 4) {
            hangArmLockCounter++;
            hangArm.setTargetPosition(hangArm.getCurrentPosition());
            if (hangArmLockCounter > 10) {
                hangArmLock.setPosition(.534);
                hangerState = 5;
            }
        } else if (hangerState == 5) {
            hangArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hangArm.setPower(0);
            hangArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangerState = 6;
        } else if (hangerState == 6) {
            hangArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangArm.setPower(0);
            hangArmLock.setPosition(.534);
        }
    }

    public void ScoringAndCollectingButtons() {
        if (gamepad2.a) {
            if (buttonAToggle == false && buttonAWasPressed == false) {
                buttonAToggle = true;
            } else if (buttonAToggle == true && buttonAWasPressed == false) {
                buttonAToggle = false;
            }
            buttonAWasPressed = true;

        } else {
            buttonAWasPressed = false;
        }
        if (buttonAToggle) {
            scoreInLander();
        } else {
            craterState = 0;
            landerState = 0;
            moveToBlocks = false;
            ballInLeft = false;
            ballInRight = false;
            bumperPressedLander = false;
        }
    }

    public void scoreInLander() {
        armMoved = true;
        if (bumperPressedLander && !gamepad2.right_bumper) {
            bumperPressedLander = false;
        }
        if (landerState == 0) {//1550 middle motor change
            startingAngleLander = Double.parseDouble(angleDouble);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor.setPower(0);
            middleMotor2.setPower(0);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /*if(firstAutoButton = false) {
                priorShoulderPos = shoulderMotor.getCurrentPosition();
                priorElbowPos = elbowMotor.getCurrentPosition();
                priorRotationPos = rotationMotor.getCurrentPosition();
            }
            firstAutoButton = true;*/
            rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationMotor.setTargetPosition(1400);
            shoulderMotor.setTargetPosition(1754);
            elbowMotor.setTargetPosition(1732);
            shoulderMotor.setPower(.8);
            elbowMotor.setPower(.8);
            nowTurning = false;
            landerState = 1;
        }
        if (landerState == 1) {
            if (ExcessStuff.closeEnough(shoulderMotor.getCurrentPosition(), 1754, 25) && ExcessStuff.closeEnough(elbowMotor.getCurrentPosition(), 1732, 25)) {
                landerState = 2;
                rotationMotor.setPower(.7);
            }
        }
        if (landerState == 2) {
            if (ExcessStuff.closeEnough(rotationMotor.getCurrentPosition(), 1400, 50)) {
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                middleMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                /*if(sideAdjusted!= 422422) {
                    leftMotor.setTargetPosition(sideAdjusted);
                    rightMotor.setTargetPosition(sideAdjusted);
                }
                else {*/
                leftMotor.setTargetPosition(3200);
                rightMotor.setTargetPosition(3200);
                /*}
                if(middleAdjusted != 422422) {
                    middleMotor.setTargetPosition(middleAdjusted);
                    middleMotor2.setTargetPosition(middleAdjusted);
                }
                else {*/
                middleMotor.setTargetPosition(1550);
                middleMotor2.setTargetPosition(1550);
                //}
                leftMotor.setPower(.6);
                rightMotor.setPower(.6);
                middleMotor.setPower(.6);
                middleMotor2.setPower(.6);
                landerState = 3;
                dpadWasPressed = false;
                firstTimeGeneral = true;
                reachedGoal = false;
            }
        }
        if (landerState == 3) {
            if (firstTimeGeneral) {
                if (leftMotor.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
                    powerMultiplier = ((startingAngle - Double.parseDouble(angleDouble)) / 90);
                    leftMotor.setPower((.6 - powerMultiplier));
                    rightMotor.setPower((.6 + powerMultiplier));
                }
            }
            if ((ExcessStuff.closeEnough(leftMotor.getCurrentPosition(), 3200, 25) || ExcessStuff.closeEnough(leftMotor.getCurrentPosition(), sideAdjusted, 25)) && (ExcessStuff.closeEnough(middleMotor.getCurrentPosition(), 1550, 25) || ExcessStuff.closeEnough(middleMotor.getCurrentPosition(), middleAdjusted, 25))) {
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (firstTimeGeneral) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    middleMotor2.setPower(0);
                    middleMotor.setPower(0);
                    firstTimeGeneral = false;
                }
                reachedGoal = true;
            }
            if ((!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) && reachedGoal) {
                powerMultiplier = 2 * ((startingAngle - Double.parseDouble(angleDouble)) / 90);
                leftMotor.setPower(-powerMultiplier);
                rightMotor.setPower(powerMultiplier);
            }
            if (gamepad2.right_bumper) {
                landerState = 4;
                bumperPressedLander = true;
            }
            if (gamepad2.left_trigger == 1) {
                leftIntake.setPosition(.05);
            } else {
                leftIntake.setPosition(.5);
            }
            if (gamepad2.right_trigger == 1) {
                rightIntake.setPosition(.05);
            } else {
                rightIntake.setPosition(.5);
            }
        }
        if (landerState == 4) {
            if (dpadWasPressed) {
                sideAdjusted = leftMotor.getCurrentPosition();
                middleAdjusted = middleMotor.getCurrentPosition();
            }
            dpadWasPressed = false;
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(1280);
            elbowMotor.setTargetPosition(1728);
            rotationMotor.setTargetPosition(1500);
            landerState = 5;
            firstTimeGeneral = true;
        }
        if (landerState == 5) { //empties the right ones
            if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right && !gamepad1.dpad_left) {
                powerMultiplier = ((startingAngle - Double.parseDouble(angleDouble)) / 90);
                leftMotor.setPower((-powerMultiplier));
                rightMotor.setPower((powerMultiplier));
            }
            if (gamepad2.left_trigger == 1) {
                leftIntake.setPosition(.05);
            } else {
                leftIntake.setPosition(.5);
            }
            if (gamepad2.right_trigger == 1) {
                rightIntake.setPosition(.05);
            } else {
                rightIntake.setPosition(.5);
            }
            if (gamepad2.right_bumper && bumperPressedLander == false) {
                landerState = 6;
            }
        }
        if (landerState == 6) {
            dpadWasPressed = false;
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPosition(300);
            rightMotor.setTargetPosition(300);
            middleMotor.setTargetPosition(-150);
            middleMotor2.setTargetPosition(-150);
            leftMotor.setPower(.5);
            rightMotor.setPower(.5);
            middleMotor.setPower(.3);
            middleMotor2.setPower(.3);
            landerState = 7;
        }
        if (landerState == 7) {
            powerMultiplier = ((startingAngle - Double.parseDouble(angleDouble)) / 90);
            leftMotor.setPower((.5 + powerMultiplier));
            rightMotor.setPower((.5 - powerMultiplier));
            if (ExcessStuff.closeEnough(leftMotor.getCurrentPosition(), 1000, 100) || ExcessStuff.closeEnough(rightMotor.getCurrentPosition(), 1000, 100)) {
                rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotationMotor.setTargetPosition(priorRotationPos);
                rotationMotor.setPower(.6);
                landerState = 8;
            }
        }
        if (landerState == 8) {
            if (rotationMotor.getCurrentPosition() > 2700) {
                rotationMotor.setPower(.25);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                middleMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setPower(-.3);
                rightMotor.setPower(-.3);
                middleMotor.setPower(0);
                middleMotor2.setPower(0);
                elbowMotor.setTargetPosition(priorElbowPos);
                shoulderMotor.setTargetPosition(priorShoulderPos);
                elbowMotor.setPower(.6);
                shoulderMotor.setPower(.6);
                rotationMotor.setPower(ExcessStuff.scaleSpeed(.4, .1, priorRotationPos, rotationMotor.getCurrentPosition()));
            }
        }

    }

    public void setZeros() {
        if (armMoved == false) {
            rotationMotor.setPower(0);
        }
    }

    public void checkAutoArmButtons() {
        /*if(gamepad1.a) { //lander
            armMoved = true;
            if(firstTimeA2) {
                shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulderMotor.setTargetPosition(1985);
                elbowMotor.setTargetPosition(1228);
                rotationMotor.setTargetPosition(-500);
                shoulderMotor.setPower(.7);
                elbowMotor.setPower(.7);
                rotationMotor.setPower(.5);
                firstTimeA2 = false;
            }
            wasPressed2 = true;
        }
        else if(gamepad1.y) { //crater
            armMoved = true;
            if(firstTimeY2) {
                shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulderMotor.setTargetPosition(2904);
                elbowMotor.setTargetPosition(-1200);
                shoulderMotor.setPower(.6);
                elbowMotor.setPower(.6);
                firstTimeY2 = false;
            }
            wasPressed2 = true;
        }
        else {
            if(wasPressed2) {
                shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //1985s
                //1228 e
                wasPressed2 = false;
            }
            firstTimeA2 = true;
            firstTimeY2 = true;
        }*/
    }

    public void checkDeadReckonButton() {
        if (gamepad2.b) {
            firstTimeNotSeeingB = true;
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPosition(0);
            rightMotor.setTargetPosition(0);
            middleMotor.setTargetPosition(0);
            middleMotor2.setTargetPosition(0);
        } else {
            if (firstTimeNotSeeingB) {
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                firstTimeNotSeeingB = false;
            }
        }
    }

    public void checkEncoderModes() {
        if ((!leftMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) || !rightMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) || !middleMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)) && !buttonAToggle && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad2.b) {
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
