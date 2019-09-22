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
        double newAngle = inputAngle + 180;
        return newAngle;
    }
    static public void setSpeedMode() {
    }

}
