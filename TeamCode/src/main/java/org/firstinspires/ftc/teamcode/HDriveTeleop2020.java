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

@TeleOp(name = "SkyStone TeleOp", group = "HDrive")
public class HDriveTeleop2020 extends OpMode {
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

    boolean fieldCentric = true;
    boolean isGettingOnLander = false;
    boolean isPressedX = false;
    boolean isFieldCentricButton = false;
    boolean armMode = false;
    boolean sideMoved = false;
    boolean dpadWasPressed = false;
    boolean updatedDpadMovement = false;
    boolean rightStickMoved = false;
    float rightX;
    long rightStickTimer = 0;
    boolean newAPressed2 = true;
    boolean newYPressed2 = true;
    int armPos = 0;
    double goalPos;


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
        leftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor2");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");
        rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor2");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        armFlipper = (DcMotorEx) hardwareMap.get(DcMotor.class, "armFlipper");
        calculator = new HDriveFCCalc();
        dpadCalculator = new HDriveFCCalc();
        armCalculator = new ArmCalculator(bicep, forearm);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);

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
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 10;
        pidStuff.i = 100;
        pidStuff.d = 0;
        pidStuff.f = 14;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        leftMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);

    timeDifferenceBetweenLoops = System.currentTimeMillis();
}

    public void loop() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle));
        //telemetry.addData("Hold Angle", ExcessStuff.convertAngle(holdAngle));
        //telemetry.addData("Current Angle", ExcessStuff.convertAngle(angleDouble));

        dpadCheck();
        checkFieldCentricAndSlowMode();
        moveTheBase();
        checkEncoderModes();
        tankDriveOdometry();
        moveArm();
        telemetry.addData("Left Move", calculator.getLeftDrive());
        telemetry.addData("Right Move", calculator.getRightDrive());
        telemetry.addData("Middle Move", calculator.getMiddleDrive());
        telemetry.addData("Angle", angleDouble);
        telemetry.addData("Middle Power", middleMotor.getPower());
        telemetry.addData("Left 2 Power", leftMotor2.getPower());
        telemetry.update();
    }

    public void dpadCheck() {

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
            } else if (rightStickMoved && rightX == 0){
                double timeDifference = System.currentTimeMillis() - rightStickTimer;
                holdAngle = angleDouble;
                if(timeDifference > 500) {
                    rightStickMoved = false;
                }
            }
            if (gamepad1.b == true) {
                offset = angleDouble + 180;
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
                calculator.calculateMovement(leftX, leftY, rightX, angleDouble + offset);
            } else {
                calculator.calculateMovement(-leftX, -leftY, rightX, 0);
            }
            if (calculator.getLeftDrive() != 0 || calculator.getRightDrive() != 0 || calculator.getMiddleDrive() != 0) {
                sideMoved = true;
            }
            maintainAngle();
            leftMotor.setPower(speed * calculator.getLeftDrive() /*+ sideChangePower*/);
            leftMotor2.setPower(speed * calculator.getLeftDrive() /*+ sideChangePower*/);
            rightMotor.setPower(speed * calculator.getRightDrive() /*- sideChangePower*/);
            rightMotor2.setPower(speed * calculator.getRightDrive() /*- sideChangePower*/);
            middleMotor.setPower(speed * calculator.getMiddleDrive() * 2);
        }
    }
    public void tankDriveOdometry() {
        here++;
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(extraClasses.formatAngle(angles.angleUnit, angles.firstAngle));
        double leftPosChange = leftMotor.getCurrentPosition() - lastLeftPos;
        double rightPosChange = rightMotor.getCurrentPosition() - lastRightPos;
        lastLeftPos = leftMotor.getCurrentPosition();
        lastRightPos = rightMotor.getCurrentPosition();
        xPos = (xPos + ((leftPosChange + rightPosChange)/2) * Math.sin(Math.toRadians(extraClasses.convertAngle(angleDouble))));
        yPos = (yPos + ((leftPosChange + rightPosChange)/2) * Math.cos(Math.toRadians(extraClasses.convertAngle(angleDouble))));
        /*telemetry.addData("XPos", xPos);
        telemetry.addData("YPos", yPos);
        telemetry.addData("Angle", extraClasses.convertAngle(angleDouble));
        telemetry.addData("loops", here);*/
        telemetry.update();
    }
    public void maintainAngle() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = Double.parseDouble(ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle));
        sideChangePower = (extraClasses.convertAngle(angleDouble) - extraClasses.convertAngle(holdAngle))/75;
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
    public void moveArm(){
        //closes claw and moves arm all the way over the robot/ back to home position
        if (gamepad2.y && newYPressed2){
            newYPressed2 = false;
            switch(armPos) {
                case 0 :
                    // currently home
                    goalPos = 1;
                    clawServo.setPosition(goalPos);
                    armFlipper.setTargetPosition(1);
                    break;
                case 1 :
                    // currently extended
                    goalPos = 0;
                    clawServo.setPosition(goalPos);
                    armFlipper.setMode(RUN_TO_POSITION);
                    armFlipper.setTargetPosition(0);
                    break;
                 default :
                     telemetry.addData("Something went wrong and it is all darby's fault", 0);
            }

            armPos += 1;
            armPos = armPos%1;
        } else{

            newYPressed2 = false;
            if (armFlipper.isBusy()){
                switch(armPos) {
                    case 0 :
                        // trying to go home
                        armFlipper.setPower(-1);
                        break;
                    case 1 :
                        // trying to extend
                        armFlipper.setPower(1);
                        break;
                    default :
                        telemetry.addData("Something went wrong and it is all darby's fault", 1);
                }
            }
        }
        if (!armFlipper.isBusy() && gamepad2.right_stick_x != 0) {
            armFlipper.setPower(gamepad2.right_stick_x);
        }
        if(gamepad2.a && newAPressed2){
            if (closeEnough (clawServo.getPosition(), /* open*/ 1, .05)){
                clawServo.setPosition(/* closed*/0);
            } else if (closeEnough (clawServo.getPosition(), /* closed*/ 0, .05)){
                clawServo.setPosition(/* open*/1);
            } else{
                telemetry.addData("Something went wrong and it is all darby's fault", 2);
            }
            newAPressed2 = false;
        }
        else{
            newAPressed2 = true;
        }

    }
    public boolean closeEnough (double current, double target, double tolerance){
        return Math.abs(current - target) <= tolerance;
    }
}
