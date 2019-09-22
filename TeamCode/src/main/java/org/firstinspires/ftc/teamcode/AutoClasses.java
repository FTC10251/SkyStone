package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

public class AutoClasses {
    static final double     COUNTS_PER_MOTOR_REV    = 1150 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4; ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = /*(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*/ 91.125;
    static final double COUNTS_PER_INCH_SIDE = 125;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    DcMotor middleMotor2;
    DcMotor shoulderMotor;
    DcMotor elbowMotor;
    DcMotor rotationMotor;
    Orientation angles;
    String angleDouble = "0";
    BNO055IMU imu;
    public AutoClasses(DcMotor leftMotorInit, DcMotor rightMotorInit, DcMotor middleMotorInit, DcMotor middleMotor2Init, DcMotor shoulderMotorInit, DcMotor elbowMotorInit, DcMotor rotationMotorInit, BNO055IMU imuInit) {
        leftMotor = leftMotorInit;
        rightMotor = rightMotorInit;
        middleMotor = middleMotorInit;
        middleMotor2 = middleMotor2Init;
        shoulderMotor = shoulderMotorInit;
        elbowMotor = elbowMotorInit;
        rotationMotor = rotationMotorInit;
        imu = imuInit;
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
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            if(runThroughs > 8) {
                atHere = true;
                angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
                endingAngle = Double.parseDouble(angleDouble);
                angleError = endingAngle - startingAngle;
                sideChange = (angleError * 6)/100;
                leftMotor.setPower(sidePower /*+ sideChange*/);
                rightMotor.setPower(sidePower/* - sideChange*/);
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
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
