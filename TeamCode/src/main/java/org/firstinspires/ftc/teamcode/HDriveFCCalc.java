package org.firstinspires.ftc.teamcode;

/**
 * Created by Luke on 9/7/2016.
 */

public class HDriveFCCalc  {
    double leftMove;
    double rightMove;
    double middleMove;
    double tempVal;
    double newVal;
    public void calculateMovement (double joystickX, double joystickY, double turnStickX, double angle) {
        double robotAngle;
        if(angle == 180) {
            robotAngle = 0;
        }
        else {
            robotAngle = -angle / 360 * 2 * Math.PI;
        }
        newVal = robotAngle;
        tempVal = joystickY * Math.cos(robotAngle) - joystickX * Math.sin(robotAngle);
        double driveMiddle = joystickX * Math.cos(-robotAngle) - joystickY * Math.sin(-robotAngle);
        double driveLeft = tempVal + turnStickX/2;
        double driveRight = tempVal - turnStickX/2;
        if (driveLeft > 1) {
            driveRight = driveRight * (1 / driveLeft);
            driveMiddle = driveMiddle * (1 / driveLeft);
            driveLeft = 1;
        } else if (driveRight > 1) {
            driveLeft = driveLeft * (1 / driveRight);
            driveMiddle = driveMiddle * (1 / driveRight);
            driveRight = 1;
        } else if(driveMiddle > 1) {
            driveLeft = driveLeft * (1 / driveMiddle);
            driveRight = driveRight * (1 / driveMiddle);
            driveMiddle = 1;
        }
        if(joystickX == 0 && joystickY == 0 && Math.abs(turnStickX) > .8) {
            driveLeft = tempVal + turnStickX;
            driveRight = tempVal - turnStickX;
        }
        leftMove = driveLeft;
        rightMove = driveRight;
        middleMove = driveMiddle;
    }
    public double isStupid(){ return tempVal; }
    public double getLeftDrive(){
        return leftMove;
    }
    public double getRightDrive(){
        return rightMove;
    }
    public double getMiddleDrive(){
        return middleMove;
    }
}
