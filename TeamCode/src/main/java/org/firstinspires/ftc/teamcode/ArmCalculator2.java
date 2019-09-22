

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmCalculator2 {
    double elbowMotorSpeed;
    double shoulderMotorSpeed;
    double armOneLength;
    double armTwoLength;
    double targetY = 0;
    public ArmCalculator2(double lengthOne, double lengthTwo){
        armOneLength = lengthOne;
        armTwoLength = lengthTwo;
    }
    public void calculateSpeed(double joystickX, double joystickY, DcMotor shoulderMotor, DcMotor elbowMotor, double shoulderAngle, double elbowAngle, Telemetry telemetry){
        targetY = targetY + .1*joystickY;
        double currentY = armOneLength*Math.sin(Math.toRadians(shoulderAngle))-armTwoLength*Math.cos(Math.toRadians(elbowAngle));
        //double scale = (-1*(armOneLength*Math.cos(Math.toRadians(shoulderAngle)) + armTwoLength*Math.sin(Math.toRadians(elbowAngle) + Math.toRadians(shoulderAngle) - 90))/(armTwoLength*Math.sin( Math.toRadians(elbowAngle) + Math.toRadians(shoulderAngle) - 90)));
        double scale = -(double)armOneLength*Math.cos(Math.toRadians((double)shoulderAngle))/((double)armTwoLength*(double)Math.sin(Math.toRadians((double)elbowAngle/*-(90.0-(double)shoulderAngle)*/)));
        double moveAcross = joystickX;
        double moveUp = moveAcross *((double)5.2/7.0) * (scale)/* + verticalAngleVelocity*/;
        //double ratio = moveUp/moveAcross;
        moveUp = moveUp + (targetY-currentY)*.01;
        if(Math.abs(moveUp) > Math.abs(moveAcross)){
            moveAcross = moveAcross*(1/Math.abs(moveUp));
            moveUp = moveUp*(1/Math.abs(moveUp));
        }
        else{
            moveUp = moveUp*(1/Math.abs(moveAcross));
            moveAcross = moveAcross*(1/Math.abs(moveAcross));
        }


        elbowMotorSpeed = moveUp;
        shoulderMotorSpeed = moveAcross;
    }

    public double getElbowMotorSpeed() {
        return -elbowMotorSpeed;
    }

    public double getShoulderMotorSpeed() {
        return shoulderMotorSpeed;
    }



}
