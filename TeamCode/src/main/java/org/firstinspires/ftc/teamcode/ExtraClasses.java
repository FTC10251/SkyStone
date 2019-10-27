package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class ExtraClasses {
    static boolean firstTime = true;
    static double startPos;
    static boolean finished = true;
    static DcMotor leftMotor;
    static DcMotor rightMotor;
    static DcMotor middleMotor;
    static DcMotor middleMotor2;
    public ExtraClasses(DcMotor leftMot, DcMotor rightMot, DcMotor middleMot, DcMotor middleMot2) {
        leftMotor = leftMot;
        rightMotor = rightMot;
        middleMotor = middleMot;
        middleMotor2 = middleMot2;

    }
    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    static public double scaleSpeed (double maxSpeed, double minSpeed, double targetPos, double currentPos){
        if (firstTime){
            startPos = currentPos;
            firstTime = false;
            finished = false;
        }
        double speedDif = maxSpeed - minSpeed;
        double dis = targetPos - startPos;
        double midPt = startPos + dis/2;
        if(currentPos >= startPos && currentPos <= midPt){
            return minSpeed + ((currentPos- startPos)/(dis/2))*(speedDif);
        }
        else if (currentPos <= targetPos && currentPos > midPt){
            return minSpeed + ((targetPos - currentPos)/(dis/2))*(speedDif);
        }
        if (currentPos >= targetPos){
            firstTime = true;
            finished = true;
        }
        return 0;
    }
    static public boolean closeEnough(double currentPos, double targetPos, double tolerance){
        if(Math.abs(targetPos - currentPos) > tolerance){
            return false;
        }
        return true;
    }
    static double distanceBetweenPoints(double prevX, double currentX, double prevY, double currentY) {
        double length = Math.sqrt(((currentX - prevX) * (currentX - prevX)) + ((currentY - prevY) * (currentY - prevY)));
        return length;
    }
    static public double convertAngle(double inputAngle) {
        double newAngle = 0;
        if(inputAngle <= 0) {
            newAngle = Math.abs(inputAngle);
        }
        else {
            newAngle = 180 + (180 - Math.abs(inputAngle));
        }
        return newAngle;
    }
    static public double angleDistance(double currentAngle, double targetAngle) {
        double distance1 = Math.abs(targetAngle - currentAngle);
        double distance2 = Math.abs((360 - currentAngle) - targetAngle);
        double angleDistance = distance1;
        if(distance1 > distance2) {
            angleDistance = distance2;
        }
        return angleDistance;
    }
    static public double angleDistanceNoAbs(double currentAngle, double targetAngle) {
        double distance1 = targetAngle - currentAngle;
        double distance2 = (360 - currentAngle) - targetAngle;
        double angleDistance = distance1;
        if(distance1 > distance2) {
            angleDistance = distance2;
        }
        return angleDistance;
    }
    static public void setSpeedMode() {
    }

}
