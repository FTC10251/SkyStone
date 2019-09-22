package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

public class RobotMovement {
    static double worldXPosition = 0; //Robots X Pos
    static double worldYPosition = 0; // Robots Y Pos
    static double worldAngle_rad = 0; // Robots Angle
    static double movement_x = 0; // Power that gets applied to the robot left motor
    static double movement_y = 0; // Power that gets applied to the robot right motor
    static double movement_turn = 0; //Amount robot needs to turn;
    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {

        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turnSpeed;

        if(distanceToTarget < 10) {
            movement_turn = 0;
        }

    }
}
