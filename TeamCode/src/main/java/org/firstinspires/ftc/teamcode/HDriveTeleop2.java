package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Locale;
// * Created by definitly not HIRSH as he would mess it up and it would explode on 8/18/2016

@TeleOp(name= "HDriveTeleop2")
@Disabled()
public class HDriveTeleop2 extends OpMode {
    //HDrive2 calculator
    HDriveFCCalc calculator;

    //Ints, Doubles, Booleans, and Floats
    int glyphMotorState;
    int glyphResetCount = 0;
    int relicClawDelay = 0;
    int relicClawDelay2 = 0;
    int state = 0;
    int countsinceapressed = 0;
    int relicClawState = 1;
    int relicClawState2 = 1;
    int fieldCentricCounter = 0;
    int glyphUpDownCounter = 0;
    int glyphCounter = 0;
    int glyphLevel = 0;
    int relicClawCounter = 10;
    int relicClawCounter2 = 10;
    int encoder = 0;
    double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    double offset = 0;
    double yAngle = 0;
    double stateGlyph = 0;
    double position = 0;
    double scale;
    double time = System.currentTimeMillis();
    double lastLiftPosition = 0;
    double currentLiftPosition = 0;
    boolean countUp = false;
    boolean timeState = true;
    boolean fieldCentric = true;
    boolean speedMode = false;
    boolean encoderReset = false;
    boolean yPressed = false;
    boolean lezGoSlow = false;
    boolean firstY = true;
    float rightX;
    float leftX;
    float leftY;
    String angleDouble = "hi";

    //Robot Hardware
    DcMotor relicArm;
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx middleMotorNumberoDdox;
    DcMotorEx glyphMotor; //Pulley
    Servo relicClaw;
    Servo relicClaw2;
    Servo claw1;
    Servo claw2;
    Servo servo2;
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
        middleMotorNumberoDdox = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor2");
        glyphMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Hirsh is very dumb");
        relicArm = (DcMotorEx) hardwareMap.get(DcMotor.class, "Relic Arm");
        claw1 = hardwareMap.servo.get("claw1");
        claw2 = hardwareMap.servo.get("claw2");
        relicClaw = hardwareMap.servo.get("Relic Claw");
        relicClaw2 = hardwareMap.servo.get("Relic Claw 2");
        calculator = new HDriveFCCalc();
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotorNumberoDdox.setDirection(DcMotor.Direction.REVERSE);
        glyphMotor.setDirection(DcMotor.Direction.REVERSE);
        glyphMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PIDCoefficients pid = glyphMotor.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        pid.d = 0;
        glyphMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);
        glyphMotorState = 1;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotorNumberoDdox.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 5;
        pidStuff.i = .4;
        pidStuff.d = 0;
        pidStuff.f = 10.2;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        middleMotorNumberoDdox.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        telemetry.addData("pid", leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();
    }
    public void loop(){

        lastLiftPosition = currentLiftPosition;
        currentLiftPosition = glyphMotor.getCurrentPosition();
        if(relicClawCounter < 10) {
            relicClawCounter++;
        }
        if(relicClawCounter2 < 10) {
            relicClawCounter2++;
        }
        if(glyphResetCount < 100){
            glyphResetCount++;
        }
        if(glyphCounter < 10){
            glyphCounter++;
        }
        if(glyphUpDownCounter < 10){
            glyphUpDownCounter++;
        }
        if(fieldCentricCounter < 10) {
            fieldCentricCounter++;
        }
        int i = 0;
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        if(yPressed == false) {
            leftX = gamepad1.left_stick_x;
            leftY = gamepad1.left_stick_y;
        }
        if(gamepad1.a != true && yPressed == false) {
            rightX = gamepad1.right_stick_x;
        }
        if(gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0 && timeState == true) {
            timeState = false;
        }
        else if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_x == 0){
            timeState = true;
            time = System.currentTimeMillis();
        }
        if(gamepad1.right_bumper){
            claw1.setPosition(.35);
            claw2.setPosition(.58);
            stateGlyph = 1;
        }
        if(countUp){
            if(countsinceapressed < 10){
                countsinceapressed++;
            }
            else{
                countUp = false;
                countsinceapressed = 0;
            }
        }
        /*if(buttonAPressed&& !countUp) {
            countUp = true;
            if (speedMode == false) {
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotorNumberoDdox.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Mode", "Speed");
                telemetry.update();
            } else if (speedMode == true) {
                speedMode = false;
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                middleMotorNumberoDdox.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Mode", "Power");
                telemetry.update();
            }
        }
        else {
            firstY = true;
        }*/
        if(gamepad1.b == true){
            offset = Double.parseDouble(angleDouble);
            offset = -offset;
        }
        if(gamepad1.a == true) {
            lezGoSlow = true;
        }
        else {
            lezGoSlow = false;
        }
        if(gamepad1.x && fieldCentricCounter > 9) {

            fieldCentric = !fieldCentric;

            fieldCentricCounter = 0;
        }
        if(fieldCentric) {
            calculator.calculateMovement(leftX, leftY, rightX, Double.parseDouble(angleDouble) + offset);
        }
        else {
            calculator.calculateMovement(leftX, leftY, rightX, 0);
        }
        leftMotor.setPower(calculator.getLeftDrive());
        rightMotor.setPower(calculator.getRightDrive());
        middleMotor.setPower(-calculator.getMiddleDrive());
        middleMotorNumberoDdox.setPower(-calculator.getMiddleDrive());
        telemetry.addData("left", leftMotor.getPower());
        telemetry.addData("right", rightMotor.getPower());
        telemetry.addData("middle", middleMotor.getPower());
        telemetry.addData("middle 2", middleMotorNumberoDdox.getPower());
        telemetry.addData("left Motor", leftMotor.getPower());
        telemetry.addData("right Motor", rightMotor.getPower());
        telemetry.addData("Imu Angle 1", angles.firstAngle);
        telemetry.addData("Imu Angle 2", angles.toAngleUnit(AngleUnit.DEGREES).secondAngle);
        telemetry.addData("Imu Angle 3", angles.thirdAngle);
        telemetry.addData("Angle", Double.parseDouble(angleDouble) + offset);
        telemetry.addData("Angle 2", yAngle);
        telemetry.addData("Puley", glyphMotor.getCurrentPosition());
        telemetry.addData("time", System.currentTimeMillis());
        telemetry.addData("Start Time," , time);
        telemetry.addData("time state", timeState);
        telemetry.addData("Relic", relicArm.getCurrentPosition());
        telemetry.addData("Mode", leftMotor.getMode());
        telemetry.addData("Claw 1", claw1.getPosition());
        telemetry.addData("Claw 2", claw2.getPosition());
        telemetry.addData("Arm Servo", relicClaw.getPosition());
        telemetry.addData("Arm Servo 2", relicClaw2.getPosition());
        telemetry.update();
        if(gamepad1.left_bumper == true && glyphUpDownCounter == 10) {
            glyphUpDownCounter = 0;
            if(glyphLevel < 2){
                glyphLevel++;
            }
            if(glyphLevel == 0){
                position = 0;
            }
            if(glyphLevel == 1){
                position = 700;
            }
            if(glyphLevel == 2){
                position = 1370;
            }
            glyphMotor.setPower(.6);
        }
        else if(gamepad1.left_trigger == 1 && glyphUpDownCounter == 10) {
            glyphUpDownCounter = 0;
            if(glyphLevel > 0){
                glyphLevel--;
            }
            if(glyphLevel == 0){
                position = 0;
            }
            if(glyphLevel == 1){
                position = 700;
            }
            if(glyphLevel == 2){
                position = 1370;
            }
            glyphMotor.setPower(-.6);
        }
        else if(gamepad1.dpad_up) {
            position = position + 25;
            glyphMotor.setPower(.35);
        }
        else if(gamepad1.dpad_down) {
            position = position - 25;
            glyphMotor.setPower(-.2);
        }
        if(position > 1370){
            position = 1370;
        }
        if(gamepad1.y) {
            yPressed = true;
            leftX = (float).5;
            rightX = (float)-.275;
        }
        else {
            yPressed = false;
        }
        if(!encoderReset) {
            if (glyphMotorState != 1) {
                glyphMotorState = 1;
                glyphMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            glyphMotor.setTargetPosition((int) position);
        }
        else{
            if(glyphMotorState != 2){
                glyphMotorState = 2;
                glyphMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(Math.abs(currentLiftPosition-lastLiftPosition) < 10&& glyphResetCount == 100){
                glyphMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                glyphResetCount = 0;
                encoderReset = false;
            }
        }
        if(stateGlyph == 0 && gamepad1.right_trigger == 1) {
            if(glyphCounter >= 10) {
                stateGlyph = 1;
                claw1.setPosition(.7);
                claw2.setPosition(.31);
                glyphCounter = 0;
            }
        }
        if(stateGlyph == 1 && gamepad1.right_trigger == 1) {
            if (glyphCounter >= 10) {
                claw1.setPosition(.9);
                claw2.setPosition(.21);
                stateGlyph = 0;
                glyphCounter = 0;
            }
        }
        if(gamepad2.back) {
            glyphMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            glyphMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(gamepad2.a && relicClawCounter > 9 && relicClawState == 0) {
            relicClaw.setPosition(.75);
            relicClawCounter = 0;
            relicClawState = 1;
        }
        else if(gamepad2.a && relicClawCounter > 9 && relicClawState == 1){
            relicClaw.setPosition(.4);
            relicClawCounter = 0;
            relicClawState = 0;
        }
        else if(gamepad2.b && relicClawCounter2 > 9 && relicClawState2 == 0) {
            relicClaw2.setPosition(.2);
            relicClawCounter2 = 0;
            relicClawState2 = 1;
        }
        else if(gamepad2.b && relicClawCounter2 > 9 && relicClawState2 == 1) {
            relicClaw2.setPosition(.6);
            relicClawCounter2 = 0;
            relicClawState2 = 0;
        }
        else if(gamepad2.right_bumper) {
            relicArm.setPower(.25);
        }
        else if(gamepad2.right_trigger == 1) {
            relicArm.setPower(-.25);
        }
        else {
            state = 1;
            relicClawDelay = 0;
            relicClawDelay2 = 0;
            relicArm.setPower(0);
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

