package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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


@TeleOp(name = "SkyStone TeleOp Updated", group = "HDrive")
public class TeleOp2020NewDrivebase extends LinearOpMode {
    HDriveFCCalc calculator;
    HDriveFCCalc dpadCalculator;
    ArmCalculator armCalculator;
    ExtraClasses extraClasses;
    //FollowPath followPath;

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
    double leftIntakeServoPosition = .35;
    double rightIntakeServoPosition = .65;

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
    boolean intakeMode = false;
    boolean leftBumperWasPressed = false;

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


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 10;
        pidStuff.i = 2;
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
            //gamepadMovements();
            //checkFieldCentricAndSlowMode();
            moveTheBase();
            moveFoundationOutOfDepot();
            //checkEncoderModes();
            //tankDriveOdometry();
            //autoScoreMode();
            //moveArm();
            controlIntake();
            pathFindingButton();
            trackVelocityandLocation();
            timeDifferencePosition = (System.currentTimeMillis() - timeBefore)/1000;
            velX = velX + (imu.getAcceleration().xAccel * timeDifferencePosition);
            velY = velY + (imu.getAcceleration().yAccel * timeDifferencePosition);
            posX = posX + velX * timeDifferencePosition;
            posY = posY + velY * timeDifferencePosition;
            timeBefore = System.currentTimeMillis();
            telemetry.addData("Adjusted Angle", extraClasses.convertAngle(angleDouble));
            telemetry.addData("Auto Scoring Mode", autoScoringMode);
            telemetry.addData("Back Distance", rangeSensorBack.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", rangeSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Y", angleDouble + offset);
            telemetry.addData("Acceleration", imu.getAcceleration());
            telemetry.addData("Block Pos X", blockPosX);
            telemetry.addData("Block Pos Y", blockPosY);
            telemetry.addData("angles", angleDouble);
            telemetry.addData("Foundation State", foundationState);
            telemetry.update();
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
                leftMotor.setPower(dpadCalculator.getLeftDrive());
                leftMotor2.setPower(dpadCalculator.getLeftDrive());
                rightMotor.setPower(dpadCalculator.getRightDrive());
                rightMotor2.setPower(dpadCalculator.getRightDrive());
            } else if ((leftMotor.getPower() != 0 || rightMotor.getPower() != 0 || middleMotor.getPower() != 0) && updatedDpadMovement) {
                leftMotor.setPower(0);
                leftMotor2.setPower(0);
                rightMotor.setPower(0);
                rightMotor2.setPower(0);
                middleMotor.setPower(0);
                updatedDpadMovement = false;
            }
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
                holdAngle = angleDouble + offset;
                rightStickMoved = true;
                rightStickTimer = System.currentTimeMillis();
            } else if (rightStickMoved && rightX == 0) {
                double timeDifference = System.currentTimeMillis() - rightStickTimer;
                holdAngle = angleDouble + offset;
                if (timeDifference > 500) {
                    rightStickMoved = false;
                }
            }
            if (gamepad1.b == true) {
                offset = angleDouble + 180;
                offset = -offset;
            }


            calculator.calculateMovement(leftX, leftY, rightX, angleDouble + offset);

            if (calculator.getLeftDrive() != 0 || calculator.getRightDrive() != 0 || calculator.getMiddleDrive() != 0) {
                sideMoved = true;
            }
            maintainAngle();
            if (driverHasControl) {
                leftMotor.setPower(speed * calculator.getLeftDrive() /*+ sideChangePower*/);
                leftMotor2.setPower(speed * calculator.getLeftDrive() /*+ sideChangePower*/);
                rightMotor.setPower(speed * calculator.getRightDrive() /*- sideChangePower*/);
                rightMotor2.setPower(speed * calculator.getRightDrive() /*- sideChangePower*/);
                middleMotor.setPower(speed * calculator.getMiddleDrive() * 4);
            }
            driverHasControl = true;
        }
    }

    public void tankDriveOdometry() {
        here++;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        double leftPosChange = (leftMotor.getCurrentPosition()/COUNTS_PER_INCH) - lastLeftPos;
        double rightPosChange = (rightMotor.getCurrentPosition()/COUNTS_PER_INCH) - lastRightPos;
        double midPosChange = ((-1 * middleMotor.getCurrentPosition())/COUNTS_PER_INCH) - lastMidPos;
        lastLeftPos = leftMotor.getCurrentPosition()/COUNTS_PER_INCH;
        lastRightPos = rightMotor.getCurrentPosition()/COUNTS_PER_INCH;
        lastMidPos = (-1 * middleMotor.getCurrentPosition())/COUNTS_PER_INCH;
        xPos = (xPos + ((leftPosChange + rightPosChange) / 2) * Math.sin(Math.toRadians(extraClasses.convertAngle(angleDouble))) + (midPosChange) * Math.cos(Math.toRadians(extraClasses.convertAngle(angleDouble))));
        yPos = (yPos + ((leftPosChange + rightPosChange) / 2) * Math.cos(Math.toRadians(extraClasses.convertAngle(angleDouble))) + (midPosChange) * Math.sin(Math.toRadians(extraClasses.convertAngle(angleDouble))));
    }

    public void maintainAngle() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        sideChangePower = (extraClasses.convertAngle(angleDouble) - extraClasses.convertAngle(holdAngle)) / 75;
    }

    public void checkEncoderModes() {
        if ((!leftMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) || !rightMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) || !middleMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER))) {
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
        if (gamepad1.a && aWasPressed == false) {
            aWasPressed = true;
            if (autoScoringMode == 0) {
                autoScoringMode = 1;
            } else if (autoScoringMode == 1) {
                autoScoringMode = 2;
            } else if (autoScoringMode == 2) {
                autoScoringMode = 0;
            }

        } else if (gamepad1.a != true) {
            aWasPressed = false;
            autoScoreState = 0;
        }

        //Driver does not have control of robot here
        if (autoScoringMode == 0 || autoScoringMode == 1) {
            if (autoScoringModeFirstTime) {
                autoScoringModeFirstTime = false;
                startingAngle = extraClasses.convertAngle(angleDouble);
            }

            if(autoScoreState == 0) {
                //Find Angle Error
                double goalAngle = 90; //Just the starting angle I think
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
                double setDistanceItShoudldBeBack = 18;
                double rangeSensorDistanceBack = rangeSensorBack.getDistance(DistanceUnit.CM);
                double distanceDifferenceBack = rangeSensorDistanceBack - setDistanceItShoudldBeBack;
                double frontPowerError = distanceDifferenceBack / 50;

                //Find how far from the foundation it is
                double setDistanceItShoudldBeMid = 50;
                if (filteredValues) {
                    rangeValuePrior = rangeValuePrior;
                } else {
                    rangeValuePrior = rangeSensorDistanceMid;
                }
                rangeSensorDistanceMid = rangeSensorLeft.getDistance(DistanceUnit.CM);
                double rangeSensorValueUsed = filterValues(rangeSensorDistanceMid, rangeValuePrior, 10);
                double distanceDifferenceMid = rangeSensorValueUsed - setDistanceItShoudldBeMid;
                double middlePowerError = distanceDifferenceMid / 40;

                leftMotor.setPower(-frontPowerError + angleAdjustPower);
                leftMotor2.setPower(-frontPowerError + angleAdjustPower);
                rightMotor.setPower(-frontPowerError + -angleAdjustPower);
                rightMotor2.setPower(-frontPowerError + -angleAdjustPower);
                middleMotor.setPower(middlePowerError);

                //COMMENT THIS OUT BRUH
                if(extraClasses.closeEnough(angleDouble, goalAngle, 3) && Math.abs(distanceDifferenceMid) < 2 && Math.abs(distanceDifferenceBack) < 2) {
                    /*armFlipper.setMode(RUN_TO_POSITION);
                    armFlipper.setTargetPosition((int)goalArmPos);
                    armFlipper.setPower(.5);
                    if(!armFlipper.isBusy()) {
                        autoScoreState = 1;
                    }*/
                }
            } else if(autoScoreState == 1) {
                //armFlipper.setPower(0);
                //clawServo.setPosition(0);
                autoScoreState = 2;
            }
        } else {
            autoScoringModeFirstTime = true;
        }
        if(autoScoringMode == 1) {
            if (gamepad1.dpad_left) {
                blockPosX = 0;
            } else if (gamepad1.dpad_right) {
                blockPosX = 1;
            }
            if (gamepad1.dpad_up && dpadWasPressed == false) {
                dpadWasPressed = true;
                blockPosY++;
            } else if (gamepad1.dpad_down && dpadWasPressed == false) {
                dpadWasPressed = true;
                blockPosY = blockPosY - 1;
            } else if(!gamepad1.dpad_down && !gamepad1.dpad_up) {
                dpadWasPressed = false;
            }
            if(gamepad1.left_trigger > .5 && leftTriggerPressed == false) {
                leftTriggerPressed = true;
                if(rotation == 0) {
                    rotation = 1;
                } else {
                    rotation = 0;
                }
            } else if(gamepad1.left_trigger < .5) {
                leftTriggerPressed = false;
            }
        }

    }
    public void controlIntake(){
        if(gamepad1.left_bumper && leftBumperWasPressed == false) {
            leftBumperWasPressed = true;
            if(intakeMode) {
                intakeMode = false;
            } else {
                intakeMode = true;
            }
        } else if(gamepad1.left_bumper) {
            leftBumperWasPressed = false;
        }

        if(intakeMode) {
            leftIntakeMotor.setPower(1);
            rightIntakeMotor.setPower(1);
            leftIntakeServoPosition = leftIntakeServoPosition - .02;
            rightIntakeServoPosition = rightIntakeServoPosition + .02;
            if(rightIntakeServoPosition > 1) {
                rightIntakeServoPosition = .65;
                leftIntakeServoPosition = .35;
            }
        }
        leftIntakeServo.setPosition(leftIntakeServoPosition);
        rightIntakeServo.setPosition(rightIntakeServoPosition);
    }

    public double filterValues(double currentValue, double previousValue, double tolerance) {
        if(Math.abs(currentValue - previousValue) < tolerance) {
            filteredValues = false;
            return currentValue;
        } else {
            filteredValues = true;
            return previousValue;
        }
    }
    public void moveFoundationOutOfDepot() {
        if (gamepad1.y && yWasPressed == false) {
            yWasPressed = true;
            if (autoMovingFoundation == false) {
                autoMovingFoundation = true;
            } else {
                autoMovingFoundation = false;
                foundationState = 0;
            }

        } else if (gamepad1.y != true) {
            yWasPressed = false;
        }

        if(autoMovingFoundation) {
            driverHasControl = false;
            if (foundationState == 0) {
                double goalAngle = 0;
                double distance1 = Math.abs(extraClasses.convertAngle(angleDouble) - goalAngle);
                double distance2 = Math.abs(Math.abs((360 - extraClasses.convertAngle(angleDouble))) - goalAngle);
                double angleError = distance1;
                double sideChange = 0;
                if (distance1 > distance2) {
                    angleError = distance2;
                }
                if ((goalAngle - extraClasses.convertAngle(angleDouble) + 360) % 360 < 180) {
                    angleError = angleError;
                } else {
                    angleError = angleError * -1;
                }
                sideChange = angleError / 100;
                if(sideChange > .3) {
                    sideChange = .3;
                }


                double setDistanceItShoudldBeMid = 30;
                if (filteredValues) {
                    rangeValuePrior = rangeValuePrior;
                } else {
                    rangeValuePrior = rangeSensorDistanceMid;
                }
                rangeSensorDistanceMid = rangeSensorLeft.getDistance(DistanceUnit.CM);
                double rangeSensorValueUsed = filterValues(rangeSensorDistanceMid, rangeValuePrior, 10);
                double distanceDifferenceMid = rangeSensorDistanceMid - setDistanceItShoudldBeMid;
                double middlePowerError = distanceDifferenceMid / 40;
                leftMotor.setPower(sideChange);
                leftMotor2.setPower(sideChange);
                rightMotor.setPower(-sideChange);
                rightMotor2.setPower(-sideChange);
                middleMotor.setPower(-middlePowerError);

                if (extraClasses.closeEnough(angleDouble, goalAngle, 3) && extraClasses.closeEnough(rangeSensorDistanceMid, setDistanceItShoudldBeMid, 3)) {
                    foundationState = 1;
                    leftMotor.setPower(0);
                    leftMotor2.setPower(0);
                    rightMotor.setPower(0);
                    rightMotor2.setPower(0);
                    middleMotor.setPower(0);
                    //hookServo.setPosition(0);
                    startingTime = System.currentTimeMillis();
                }
            } else if (foundationState == 1) {
                timeDifference = System.currentTimeMillis() - startingTime;
                if (timeDifference > 1000) {
                    double angleItShouldBe = 0;
                    double distance1 = Math.abs(angleDouble - angleItShouldBe);
                    double distance2 = Math.abs(Math.abs((360 - angleDouble)) - angleItShouldBe);
                    double angleError = distance1;
                    if (distance1 > distance2) {
                        angleError = distance2;
                    }
                    if ((angleItShouldBe - angleDouble + 360) % 360 < 180) {
                        angleError = angleError;
                    } else {
                        angleError = angleError * -1;
                    }
                    angleError = angleError / 50;

                    double setDistanceItShouldBe = 10;
                    double rangeSensorDistanceBack = rangeSensorBack.getDistance(DistanceUnit.CM);
                    double sidePower = (rangeSensorDistanceBack / setDistanceItShouldBe) / 40;
                    if(sidePower > .5) {
                        sidePower = .5;
                    }

                    leftMotor.setPower(-sidePower - angleError);
                    leftMotor2.setPower(-sidePower - angleError);
                    rightMotor.setPower(-sidePower + angleError);
                    rightMotor2.setPower(-sidePower + angleError);

                    if (rangeSensorDistanceBack < setDistanceItShouldBe) {
                        //hookServo.setPosition(1);
                        startingTime = System.currentTimeMillis();
                        foundationState = 2;
                        leftMotor.setPower(0);
                        leftMotor2.setPower(0);
                        rightMotor.setPower(0);
                        rightMotor2.setPower(0);
                    }
                }
            }  else if(foundationState == 2) {
                timeDifference = System.currentTimeMillis() - startingTime;
                if(timeDifference > 500) {
                    leftMotor.setPower(.2);
                    leftMotor2.setPower(.2);
                    rightMotor.setPower(.2);
                    rightMotor2.setPower(.2);
                }
            }
        }
    }
    public void pathFindingButton() {
        if(gamepad1.x && xWasPressed == false) {
            xWasPressed = true;
            if(pathFindingMode) {
                pathFindingMode = false;
            } else {
                pathFindingMode = true;
            }
        } else if(!gamepad1.x) {
            xWasPressed = false;
            pathFindingModeFirstTime = true;
        }

        if(pathFindingMode) {
            if(pathFindingModeFirstTime) {
                pathFindingModeFirstTime = false;
                ArrayList<Point> Points = new ArrayList<>();
                //followPath.createNewPath(Points, getVelocity(), getLocation(), .75, 1, .4);
            }
            //followPath.seek();
            //leftMotor.setPower(followPath.getLeftPower());
            //leftMotor2.setPower(followPath.getLeftPower());
            //rightMotor.setPower(followPath.getRightPower());
            //rightMotor2.setPower(followPath.getRightPower());
            driverHasControl = false;

        }
    }
    public void trackVelocityandLocation() {
        //Velocity part
        double currentTime = System.currentTimeMillis();
        double timeChange = currentTime - previousTime;
        double currentSideEncoder = leftMotor.getCurrentPosition();
        double currentMidEncoder = middleMotor.getCurrentPosition();
        double currentX = currentMidEncoder * Math.cos(Math.toRadians(angleDouble) + currentSideEncoder * Math.sin(Math.toRadians(angleDouble)));
        double currentY = currentMidEncoder * Math.sin(Math.toRadians(angleDouble) + currentSideEncoder * Math.sin(Math.toRadians(angleDouble)));
        double xChange = currentX - previousX;
        double yChange = currentY - previousY;
        double xVelocity = currentX / timeChange;
        double yVelocity = currentY / timeChange;
        double velocityMagnitude = Math.sqrt(Math.pow(xChange,2) + Math.pow(yChange,2));
        velocity = new Point(velocityMagnitude,angleDouble);

        //Location part
        previousTime = currentTime;
        previousX = currentSideEncoder;
        previousY = currentMidEncoder;
    }
    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
