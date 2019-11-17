package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.ManualImports.Point;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import java.util.ArrayList;

public class FollowPath {
    static ArrayList<Point> allPoints;
    static Point velocity = new Point(0,0);
    static Point previousVelocity = new Point(0,0);
    static Point acceleration;
    static Point location;
    static double futureTime;
    static double maxSpeed;
    static double maxForce;

    static double leftPower = 0;
    static double rightPower = 0;
    public FollowPath() {

    }
    public static void createNewPath(ArrayList<Point> inputPoints, Point inputVelocity, Point inputLocation, double inputFutureTime, double inputMaxSpeed, double inputMaxForce) {
        allPoints = inputPoints;
        velocity = inputVelocity;
        location = inputLocation;
        futureTime = inputFutureTime;
        maxSpeed = inputMaxSpeed;
        maxForce = inputMaxForce;
        Point steer = findSteerAmount();
    }

    public static void seek() {
        Point steer = findSteerAmount();
        steer = MathFunctions.Limit(steer,maxForce);
        applyForce(steer);
        findVelocityChange();
    }

    public static Point findSteerAmount() {
        double worldRecordDistance = 100000;
        Point worldRecordPoint = new Point();
        for(int i = 1; i < allPoints.size(); i++) {
            double slope = MathFunctions.findSlope(allPoints.get(i-1).x, allPoints.get(i-1).y, allPoints.get(i).x, allPoints.get(i).y);
            double yIntercept = MathFunctions.findYIntercept(slope, allPoints.get(i).x, allPoints.get(i).y);
            double distance = MathFunctions.distanceBetweenPointAndLine(PredictFutureLocation(futureTime).x, PredictFutureLocation(futureTime).y, slope, yIntercept);
            Point secondPoint = MathFunctions.secondPointAlongPerpendicularLine(PredictFutureLocation(futureTime).x, PredictFutureLocation(futureTime).y, slope, yIntercept);
            if(distance < worldRecordDistance && secondPoint.x > Math.min(allPoints.get(i-1).x,allPoints.get(i).x) && secondPoint.x < Math.max(allPoints.get(i-1).x, allPoints.get(i).x)) {
                worldRecordDistance = distance;
                worldRecordPoint = secondPoint;
            }
        }
        if(worldRecordDistance == 100000) {
            for(int i = 0; i < allPoints.size(); i++) {
                double slope = MathFunctions.distanceBetween2Points(PredictFutureLocation(futureTime).x, PredictFutureLocation(futureTime).y, (int)allPoints.get(i).x, (int)allPoints.get(i).y);
                double yIntercept = MathFunctions.findYIntercept(slope, allPoints.get(i).x, allPoints.get(i).y);
                double distance = MathFunctions.distanceBetweenPointAndLine(PredictFutureLocation(futureTime).x, PredictFutureLocation(futureTime).y, slope, yIntercept);
                if(distance < worldRecordDistance) {
                    worldRecordDistance = distance;
                    worldRecordPoint = new Point(allPoints.get(i).x,allPoints.get(i).y);
                }
            }
        }
        double goalSlope = MathFunctions.findSlope(location.x,location.y,worldRecordPoint.x,worldRecordPoint.y);
        Point goalVelocity = new Point(maxSpeed, Math.toDegrees(Math.atan(goalSlope)));
        double xDiff = goalVelocity.x * Math.cos(Math.toRadians(goalVelocity.y)) - (velocity.x * Math.cos(Math.toRadians(velocity.y)));
        double yDiff = goalVelocity.x * Math.sin(Math.toRadians(goalVelocity.y)) - (velocity.x * Math.sin(Math.toRadians(velocity.y)));
        double magnitude = Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff,2));
        double angle = Math.toDegrees(Math.atan2(yDiff, xDiff));
        Point returnPoint = new Point (magnitude,angle);
        Point steer = returnPoint;

        return steer;
    }

    public static Point PredictFutureLocation(double timeInFuture) {
        Point futureLocation = new Point();
        futureLocation.x = location.x + (velocity.x * (Math.cos(Math.toRadians(velocity.y)) * timeInFuture));
        futureLocation.y = location.y + (velocity.x * (Math.sin(Math.toRadians(velocity.y)) * timeInFuture));
        return futureLocation;
    }

    public static void applyForce(Point steerForce) {
        acceleration = MathFunctions.AddVectors(steerForce, acceleration);
    }

    public static void findVelocityChange() {

    }
    public static double getLeftPower() {
        return leftPower;
    }
    public static double getRightPower() {
        return rightPower;
    }
}
