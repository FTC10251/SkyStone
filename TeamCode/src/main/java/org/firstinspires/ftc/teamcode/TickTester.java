package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "Tick Tester")
public class TickTester extends OpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1150 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4; ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = /*(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*/91.125;
    static final double COUNTS_PER_INCH_SIDE = 125;
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
    String angleDouble = "hi";
    Orientation angles;
    BNO055IMU imu;
    PIDFCoefficients pidStuff;
    boolean state = false;
    boolean isPressedX = false;
    boolean isPressedTrigger = false;
    boolean isPressedBumper = false;
    boolean runMotors = false;
    int counter = 150;
    int leftEncoders = 0;
    int rightEncoders = 0;
    int middleEncoders = 0;
    int changed = 0;

    int leftEncodersFinal = 0;
    int rightEncodersFinal = 0;
    int middleEncodersFinal = 0;

    int leftChange = 0;
    int rightChange = 0;
    int middleChange = 0;

    double leftChangeInches = 0;
    double rightChangeInches = 0;
    double middleChangeInches = 0;
    double bouncerPos = 0;

    @Override
    public void init() {
        /** Wait for the game to begin */
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
        leftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"leftMotor");
        rightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx)hardwareMap.get(DcMotor.class,"middleMotor2");
        hangArm = (DcMotorEx)hardwareMap.get(DcMotor.class, "Hang Arm");
        shoulderMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Shoulder Motor");
        elbowMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"Elbow Motor");
        rotationMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "Rotation Motor");
        hangArmLock = hardwareMap.servo.get("Hang Arm Lock");
        leftIntake = hardwareMap.get(Servo.class, "Left Intake");
        rightIntake = hardwareMap.get(Servo.class, "Right Intake");
        bouncer = hardwareMap.get(Servo.class, "Bouncer");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        hangArm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);

        elbowMotor.setDirection(DcMotor.Direction.REVERSE);
        rotationMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.left_stick_y);
        middleMotor.setPower(gamepad1.left_stick_x);
        middleMotor2.setPower(gamepad1.left_stick_x);
        if(gamepad2.x && isPressedX) {
            isPressedX = false;
            resetEncoders();
        }
        else if (!gamepad2.x){
            isPressedX = true;
        }
        if(gamepad2.y) {
            shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition());
            elbowMotor.setTargetPosition(elbowMotor.getCurrentPosition());
            runMotors = false;
        }
        else {
            runMotors = true;
        }

        if(gamepad2.left_bumper && isPressedBumper) {
            isPressedBumper = false;
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (!gamepad2.left_bumper){
            isPressedBumper = true;
        }

        if(gamepad2.left_trigger == 1 && isPressedTrigger) {
            isPressedTrigger = false;
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if (gamepad2.left_trigger != 1){
            isPressedTrigger = true;
        }
        if (gamepad2.left_bumper) {
            leftIntake.setPosition(.05);
        } else {
            leftIntake.setPosition(0.5 + (gamepad2.left_trigger / 2.3));
        }
        if (gamepad2.right_bumper) {
            rightIntake.setPosition(.05);
        } else {
            rightIntake.setPosition(0.5 + (gamepad2.right_trigger / 2.3));

        }
        if (gamepad2.dpad_up) {
            bouncerPos = bouncerPos + .03; //.58
        } else if (gamepad2.dpad_down) {
            bouncerPos = bouncerPos - .03;//.73fr
        }
        bouncer.setPosition(bouncerPos);


        /*if(gamepad1.a) {
            double leftPower = .3 + .5*Math.cos((Double.parseDouble(angleDouble)));
            double rightPower = -.3 + .5*Math.cos((Double.parseDouble(angleDouble)));
            double middlePower = .5*Math.sin(Double.parseDouble(angleDouble));
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
            middleMotor.setPower(middlePower);
        }
        telemetry.addData("left Motor", leftMotor.getCurrentPosition());
        telemetry.addData("right Motor", rightMotor.getCurrentPosition());
        telemetry.addData("middle Motor", middleMotor.getCurrentPosition());
        telemetry.addData("middle Motor 2", middleMotor2.getCurrentPosition());
        telemetry.addData("angle", angleDouble);
        double realAngle = convertAngle(angleDouble);
        telemetry.addData("Real Angle", realAngle);
        telemetry.update();*/
        if(runMotors) {
            rotationMotor.setPower(.5 * gamepad2.right_stick_x);
            elbowMotor.setPower(.5 * gamepad2.left_stick_y);
            shoulderMotor.setPower(.5 * gamepad2.left_stick_x);
        }
        if(hangArmLock.getPosition() != .62 && hangArmLock.getPosition() != .35) {
            changed++;
        }
        if(gamepad1.left_bumper) {
            hangArmLock.setPosition(.35);
        }
        else if(gamepad1.right_bumper) {
            hangArmLock.setPosition(.62);
        }
        if(gamepad1.a && state == false && counter > 150) {
            state = true;
            counter = 0;
            leftEncoders = leftMotor.getCurrentPosition();
            rightEncoders = rightMotor.getCurrentPosition();
            middleEncoders = middleMotor.getCurrentPosition();
        }
        else if(gamepad1.a && state == true && counter > 150) {
            state = false;
            counter = 0;
            leftEncodersFinal = leftMotor.getCurrentPosition();
            rightEncodersFinal = rightMotor.getCurrentPosition();
            middleEncodersFinal = middleMotor.getCurrentPosition();

            leftChange = leftEncodersFinal - leftEncoders;
            rightChange = rightEncodersFinal - rightEncoders;
            middleChange = middleEncodersFinal - middleEncoders;

            leftChangeInches = leftChange/COUNTS_PER_INCH;
            rightChangeInches = rightChange/COUNTS_PER_INCH;
            middleChangeInches = middleChange/COUNTS_PER_INCH;
            telemetry.addData("Left Change", leftChange);
            telemetry.addData("Right Change", rightChange);
            telemetry.addData("Middle Change", middleChange);
            telemetry.addData("Left Inches", leftChangeInches);
            telemetry.addData("Right Inches", rightChangeInches);
            telemetry.addData("Middle Inches", middleChangeInches);
            telemetry.update();
        }
        telemetry.addData("Left Ticks: ", leftMotor.getCurrentPosition());
        telemetry.addData("Right Ticks: ", rightMotor.getCurrentPosition());
        telemetry.addData("Hang Arm Lock", hangArmLock.getPosition());
        telemetry.addData("Changed", changed);
        telemetry.addData("Hang Arm", hangArm.getCurrentPosition());
        telemetry.addData("Middle", middleMotor.getCurrentPosition());
        telemetry.addData("Shoulder Pos", shoulderMotor.getCurrentPosition());
        telemetry.addData("Elbow Pos", elbowMotor.getCurrentPosition());
        telemetry.addData("Rotation Pos", rotationMotor.getCurrentPosition());
        telemetry.addData("Elbow Angle", elbowAngle(elbowMotor.getCurrentPosition()));
        telemetry.addData("Shoulder Angle", shoulderAngle(shoulderMotor.getCurrentPosition()));
        telemetry.addData("Side Inches", leftMotor.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("Mid Inches", middleMotor.getCurrentPosition()/COUNTS_PER_INCH_SIDE);
        telemetry.addData("Angle", angleDouble);
        telemetry.addData("Bouncer", bouncer.getPosition());

        //telemetry.addData("Counts", leftMotor.get);
        telemetry.update();
        //telemetry.addData("Counter",counter);
        //telemetry.update();
        counter++;
        if(gamepad2.dpad_up) {
            elbowMotor.setPower(.2);
        }
        else if(gamepad2.dpad_up) {
            elbowMotor.setPower(-.2);
        }
        else {
            elbowMotor.setPower(0);
        }
        if(gamepad2.dpad_right) {
            shoulderMotor.setPower(.2);
        }
        else if(gamepad2.dpad_left) {
            shoulderMotor.setPower(-.2);
        }
        else {
            shoulderMotor.setPower(0);
        }
    }
    public double elbowAngle(double currentPos) {
        double finalPos = 90-shoulderAngle((double)shoulderMotor.getCurrentPosition()) +80+ ((currentPos + (2300-1576))/(2300*4)) * 360;
        return finalPos;
    }
    public double shoulderAngle(double currentPos) {
        double finalPos = ((-currentPos+4330)/(2350*4)) * 360;
        return finalPos;
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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
}
