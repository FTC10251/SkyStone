package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.Locale;

@TeleOp(name = "Hirsh's stupid program")
public class RoboPosTune extends LinearOpMode {
    BNO055IMU imu;
    ExtraClasses extraClasses;
    Orientation angles;
    DistanceSensor rangeSensorLeft;
    DistanceSensor rangeSensorBack;
    DcMotorEx leftMotor;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor;
    DcMotorEx rightMotor2;
    DcMotorEx middleMotor;
    DcMotorEx leftIntakeMotor;
    DcMotorEx rightIntakeMotor;
    DcMotorEx arm;
    Servo clawServo;
    double angleDouble = 0;
    double differentialBackDistanceError;
    double mid = 58;
    double back = 24;
    int readingNum = 0;
    double lastReadRange = 20;
    boolean aWasPressed = false;
    boolean dontScore = false;
    double autoScoreState = 0;
    boolean autoScoringModeFirstTime = true;
    double setDistanceItShouldBeBack = 0;
    double distanceDifferenceBack = 0;
    int blockPosY = 0;
    int blockPosX = 0;
    double reading1;
    double reading2;
    double reading3;
    double reading4;
    double lastTime = 0;
    boolean virgin = true;
    double lastDistanceBack = 0;
    double totalBackDistanceError = 0;
    boolean firstT = true;
    double rotation = 0;
    boolean leftTriggerPressed = false;
    boolean dpadWasPressed = false;
    int[][] blockPos = new int[2][10];
    double autoScoringMode = 0;
    private boolean newLeftTriggerPressed = true;
    private boolean newRightTriggerPressed = true;
    private boolean newLeftBumperPressed = true;
    private boolean newRightBumperPressed = true;


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
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Left Motor Front");
        leftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "Left Motor Back");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Right Motor Front");
        rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "Right Motor Back");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Middle Motor");
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Intake Motor Right");
        leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Intake Motor Left");
        arm = (DcMotorEx) hardwareMap.get(DcMotor.class, "Arm");
        clawServo = hardwareMap.get(Servo.class, "Claw Servo");
        //rangeSensorLeft = hardwareMap.get(DistanceSensor.class, "Range Sensor Left");
        rangeSensorBack = hardwareMap.get(DistanceSensor.class, "Range Sensor Back");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        clawServo.setPosition(.15);
        waitForStart();
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = extraClasses.convertAngle(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));

            if (gamepad1.left_trigger != 0 && newLeftTriggerPressed) {
                back++;
                newLeftTriggerPressed = false;

            } else if (gamepad1.right_trigger != 0 && newRightTriggerPressed) {
                mid++;
                newRightTriggerPressed = false;
            }
            if (gamepad1.left_bumper && newLeftBumperPressed) {
                back--;
                newLeftBumperPressed = false;

            } else if (gamepad1.right_bumper && newRightBumperPressed) {
                mid--;
                newRightBumperPressed = false;
            }
            if (gamepad1.left_trigger == 0) {
                newLeftTriggerPressed = true;
            }
            if (gamepad1.right_trigger == 0) {
                newRightTriggerPressed = true;
            }
            if (!gamepad1.left_bumper) {
                newLeftBumperPressed = true;
            }
            if (!gamepad1.right_bumper) {
                newRightBumperPressed = true;
            }
            if (firstT) {
                double rangeSensorDistanceBack = rangeSensorBack.getDistance(DistanceUnit.CM);
                double filteredRangeSensorDistanceBack = filterValues(rangeSensorDistanceBack, readingNum);
                if (filteredRangeSensorDistanceBack > 40) {
                    filteredRangeSensorDistanceBack = 40;
                }
                lastDistanceBack = filteredRangeSensorDistanceBack - setDistanceItShouldBeBack;
                firstT = false;
            }

            autoScoreMode();
            telemetry.addData("PId P", distanceDifferenceBack/75);
            telemetry.addData("PID I", totalBackDistanceError);
            telemetry.addData("PID D", differentialBackDistanceError/40000);
            telemetry.addData("distance back", distanceDifferenceBack);
            telemetry.update();

        }
    }

    public void autoScoreMode() {
        //Find Angle Error
        double goalAngle = 270; //Just the starting angle I think
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
        setDistanceItShouldBeBack = 16;
        if (armAngle(arm.getCurrentPosition()) > 230) {
            double phi = 360 - armAngle(arm.getCurrentPosition());
            setDistanceItShouldBeBack = (50 * Math.cos(armAngle(arm.getCurrentPosition())) - 24);
        }


        //Find how far from the foundation it is

                    /*leftMotor.setPower(frontPowerError + angleAdjustPower);
                    leftMotor2.setPower(frontPowerError + angleAdjustPower);
                    rightMotor.setPower(frontPowerError + -angleAdjustPower);
                    rightMotor2.setPower(frontPowerError + -angleAdjustPower);*/
        //middleMotor.setPower(middlePowerError);

        //COMMENT THIS OUT BRUH
        if (extraClasses.closeEnough(angleDouble, goalAngle, 5)) {
            if (virgin){
                virgin = false;
                lastTime = System.currentTimeMillis();
                double rangeSensorDistanceBack = rangeSensorBack.getDistance(DistanceUnit.CM);
                double filteredRangeSensorDistanceBack = filterValues(rangeSensorDistanceBack, readingNum);
                lastDistanceBack = filteredRangeSensorDistanceBack - setDistanceItShouldBeBack;
            }
            double rangeSensorDistanceBack = rangeSensorBack.getDistance(DistanceUnit.CM);
            double filteredRangeSensorDistanceBack = filterValues(rangeSensorDistanceBack, readingNum);
            if (filteredRangeSensorDistanceBack > 40) {
                filteredRangeSensorDistanceBack = 40;
            }
            double time = System.currentTimeMillis();
            double deltaT = time - lastTime;
            distanceDifferenceBack = filteredRangeSensorDistanceBack - setDistanceItShouldBeBack;
            differentialBackDistanceError = (distanceDifferenceBack - lastDistanceBack) / deltaT;
            lastDistanceBack = filteredRangeSensorDistanceBack - setDistanceItShouldBeBack;
            lastTime = time;
            totalBackDistanceError += distanceDifferenceBack * deltaT/35000.0;
            double frontPowerError = -distanceDifferenceBack/ 75 - differentialBackDistanceError/40000 - totalBackDistanceError;


            leftMotor.setPower(frontPowerError + angleAdjustPower);
            leftMotor2.setPower(frontPowerError + angleAdjustPower);
            rightMotor.setPower(frontPowerError + -angleAdjustPower);
            rightMotor2.setPower(frontPowerError + -angleAdjustPower);
                        /*armFlipper.setMode(RUN_TO_POSITION);
                        armFlipper.setTargetPosition((int)goalArmPos);
                        armFlipper.setPower(.5);
                        if(!armFlipper.isBusy()) {
                            autoScoreState = 1;
                        }*/
        } else {
            leftMotor.setPower(angleAdjustPower);
            leftMotor2.setPower(angleAdjustPower);
            rightMotor.setPower(-angleAdjustPower);
            rightMotor2.setPower(-angleAdjustPower);
        }
        readingNum++;

    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    public double armAngle(double currentArmPos) {
        double angle = Math.abs(currentArmPos) / 13.333 + 50;
        return angle;
    }

    public double byeByeValue(double val1, double val2, double val3, double val4) {
        double[] values = {val1, val2, val3, val4};
        Arrays.sort(values);
        return (values[1] + values[2]) / 2;
    }
    public double filterValues(double sensorReading, int readingNum){
        for(int i = 0; i < 4; i++) {
            sensorReading = rangeSensorBack.getDistance(DistanceUnit.CM);
            if(sensorReading >= 0 && sensorReading < 256){
                i = 4;
                lastReadRange = sensorReading;
            }
        }
        RobotLog.d("rangeSensor reading: %f lastReadRange: %f", sensorReading, lastReadRange);
        return lastReadRange;
    }
}
