package org.firstinspires.ftc.teamcode.RobotControl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AutoClasses;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
@Autonomous(name = "Daniel's Autonomous")
@Disabled
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
    //DcMotorEx leftMotor2;
    DcMotorEx rightMotor;
    //DcMotorEx rightMotor2;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;
    String angleDouble = "0";
    Orientation angles;
    PIDFCoefficients pidStuff;
    boolean firstTime = true;
    double startPos;
    boolean finished = true;

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
        //leftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor2");
        //rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor2");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");

        /*
         * Initialize the drive system variables.
         * The init
         * () method of the hardware class does all the work here
         */

        telemetry.addData("Status", "Resetting Encoders for awesome reason");    //
        telemetry.update();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                //leftMotor2.getCurrentPosition(),
                rightMotor.getCurrentPosition(),
                //rightMotor2.getCurrentPosition(),
                middleMotor.getCurrentPosition());
                middleMotor2.getCurrentPosition();
        telemetry.update();
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //rightMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //leftMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor2.setDirection(DcMotor.Direction.REVERSE);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 10;
        pidStuff.i = 100;
        pidStuff.d = 0;
        pidStuff.f = 14;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        //leftMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        //rightMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        //pidStuff.f = 23;
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        initialAngle = Double.parseDouble(angleDouble);
        telemetry.addLine("Ready to Begin");
        telemetry.addData("Starting Angle", initialAngle);
        telemetry.update();

        waitForStart();
        realignRobot();
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        telemetry.addData("Current Angle", angleDouble);
        telemetry.update();
        //vision stuff
        moveBase(.2, 0, 29, 0, telemetry);
        moveBase(-.2, 0, 10, 0, telemetry);
        //intake block
        leftMotor.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        while(Double.parseDouble(angleDouble) > -90 && opModeIsActive()){
            telemetry.addData("angle", angleDouble);
            telemetry.addData("power", scaleSpeed(.2,.1,90, Double.parseDouble(angleDouble)));
            telemetry.update();
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            leftMotor.setPower(scaleSpeed2(.2,.1,90, Double.parseDouble(angleDouble)));
            rightMotor.setPower(scaleSpeed2(-.2,-.1,90, Double.parseDouble(angleDouble)));
        }
        moveBase(.2, 0, 200, 0, telemetry);
        moveBase(0,-.2, 0, 10, telemetry);
        //hook foundation
        moveBase(0,.2, 0, 20, telemetry);
    }
    public void moveBase(double sidePower, double midPower, double sideInches, double midInches, Telemetry telemetry){
        leftMotor.setMode(STOP_AND_RESET_ENCODER);
        //leftMotor2.setMode(STOP_AND_RESET_ENCODER);
        rightMotor.setMode(STOP_AND_RESET_ENCODER);
        //rightMotor2.setMode(STOP_AND_RESET_ENCODER);
        middleMotor.setMode(STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(STOP_AND_RESET_ENCODER);

        //Establish Goal Values
        double sideTicks = sideInches * COUNTS_PER_INCH * 2;
        double midTicks = midInches * COUNTS_PER_INCH_MID;

        leftMotor.setTargetPosition((int)sideTicks);
        //leftMotor2.setTargetPosition((int)sideTicks);
        rightMotor.setTargetPosition((int)sideTicks);
        //rightMotor2.setTargetPosition((int)sideTicks);
        middleMotor.setTargetPosition((int)midTicks);
        middleMotor2.setTargetPosition((int)midTicks);

        leftMotor.setPower(sidePower);
        //leftMotor2.setPower(sidePower);
        rightMotor.setPower(sidePower);
        //rightMotor2.setPower(sidePower);
        middleMotor.setPower(midPower);
        middleMotor2.setPower(midPower);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);

        double startingAngle = Double.parseDouble(angleDouble);
        double endingAngle = 0;
        double angleError = 0;

        while((leftMotor.isBusy() || rightMotor.isBusy() || middleMotor.isBusy() || middleMotor2.isBusy() && opModeIsActive())){
            telemetry.addData("Left Position : Goal", leftMotor.getCurrentPosition() + ":" + sideTicks);
            //telemetry.addData("Left2 Position : Goal", leftMotor2.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Right Position : Goal", rightMotor.getCurrentPosition() + ":" + sideTicks);
            //telemetry.addData("Right2 Position : Goal", rightMotor2.getCurrentPosition() + ":" + sideTicks);
            telemetry.addData("Middle Position : Goal", middleMotor.getCurrentPosition() + ":" + midTicks);
            telemetry.addData("Left Power", leftMotor.getPower());
            //telemetry.addData("Left Power2", leftMotor2.getPower());
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.addData("Is Busy Left", leftMotor.isBusy());
            telemetry.addData("Is Busy Mid ", middleMotor.isBusy());
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        }
        leftMotor.setPower(0);
        //leftMotor2.setPower(0);
        rightMotor.setPower(0);
        //rightMotor2.setPower(0);
        middleMotor.setPower(0);
        middleMotor2.setPower(0);
        leftMotor.setMode(RUN_USING_ENCODER);
        //leftMotor2.setMode(RUN_USING_ENCODER);
        rightMotor.setMode(RUN_USING_ENCODER);
        //rightMotor2.setMode(RUN_USING_ENCODER);
        middleMotor.setMode(RUN_USING_ENCODER);
        middleMotor2.setMode(RUN_USING_ENCODER);
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
    public double scaleSpeed (double maxSpeed, double minSpeed, double targetPos, double currentPos){
        if (firstTime){
            startPos = currentPos;
            firstTime = false;
            finished = false;
        }
        double speedDif = maxSpeed - minSpeed;
        double dis = targetPos - startPos;
        double midPt = startPos + dis/2;
        if(Math.abs(currentPos) >= Math.abs(startPos) && Math.abs(currentPos) <= Math.abs(midPt)){
            return minSpeed + ((currentPos- startPos)/(dis/2))*(speedDif);
        }
        else if (Math.abs(currentPos) <= Math.abs(targetPos) && Math.abs(currentPos) > Math.abs(midPt)){
            return minSpeed + ((targetPos - currentPos)/(dis/2))*(speedDif);
        }
        if (Math.abs(currentPos) >= Math.abs(targetPos)){
            firstTime = true;
            finished = true;
        }
        return 0;
    }
    public double scaleSpeed2 (double maxSpeed, double minSpeed, double targetPos, double currentPos){
        if (firstTime){
            startPos = currentPos;
            firstTime = false;
            finished = false;
        }
        //  if(Math.abs(currentPos - startPos) <= Math.abs(targetPos - startPos)) {
        double scale = 1 - (Math.abs(currentPos-startPos)) / (Math.abs(targetPos-startPos));
        return scale * (maxSpeed - minSpeed) + minSpeed;
        //  }
        //   return 0;
    }
}
