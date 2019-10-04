package org.firstinspires.ftc.teamcode.RobotControl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

public class Autonomous2020 extends LinearOpMode {
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();// Use a Pushbot's hardware
    static final double     COUNTS_PER_MOTOR_REV    = 1150 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4; ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = /*(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*/ 91.125;
    static final double COUNTS_PER_INCH_MID = 125;
    double initialAngle;
    DcMotorEx leftMotor;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor;
    DcMotorEx rightMotor2;
    DcMotorEx middleMotor;
    String angleDouble = "0";
    Orientation angles;
    PIDFCoefficients pidStuff;

    public void runOpMode() throws InterruptedException{
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
        leftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor2");
        rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor2");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
    }
    public void moveBase(double sidePower, double midPower, double sideInches, double midInches, Telemetry telemetry){
        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(STOP_AND_RESET_ENCODER);
        middleMotor.setMode(STOP_AND_RESET_ENCODER);

        //Establish Goal Values
        double sideTicks = sideInches * COUNTS_PER_INCH * 2;
        double midTicks = midInches * COUNTS_PER_INCH_MID;

        leftMotor.setTargetPosition((int)sideTicks);
        leftMotor2.setTargetPosition((int)sideTicks);
        rightMotor.setTargetPosition((int)sideTicks);
        rightMotor2.setTargetPosition((int)sideTicks);
        middleMotor.setTargetPosition((int)midTicks);

        leftMotor.setPower(sidePower);
        leftMotor2.setPower(sidePower);
        rightMotor.setPower(sidePower);
        rightMotor2.setPower(sidePower);
        middleMotor.setPower(midPower);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);

        double startingAngle = Double.parseDouble(angleDouble);
        double endingAngle = 0;
        double angleError = 0;

        while((leftMotor.isBusy() || leftMotor2.isBusy() || rightMotor.isBusy() || rightMotor.isBusy() || middleMotor.isBusy() &&opModeIsActive())){
            telemetry.addData("Left Position : Goal", leftMotor.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Left2 Position : Goal", leftMotor2.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Right Position : Goal", rightMotor.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Right2 Position : Goal", rightMotor2.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Middle Position : Goal", middleMotor.getCurrentPosition() + ":" + midTicks);
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.addData("Left Power2", leftMotor2.getPower());
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.addData("Is Busy Left", leftMotor.isBusy());
            telemetry.addData("Is Busy Mid ", middleMotor.isBusy());
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        }
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
        middleMotor.setPower(0);
        leftMotor.setMode(RUN_USING_ENCODER);
        leftMotor2.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        rightMotor2.setMode(RUN_USING_ENCODER);
        middleMotor.setMode(RUN_USING_ENCODER);
        telemetry.addLine("Complete");
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
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
