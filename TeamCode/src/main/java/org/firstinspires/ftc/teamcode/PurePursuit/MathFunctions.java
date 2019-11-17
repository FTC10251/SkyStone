package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.ManualImports.Point;

import java.util.ArrayList;

import static java.lang.Math.pow;

public class MathFunctions {
    /**
     * Makes sure it is in range of -180 to 180 degreees
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle) {
        while(angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
                                                          Point linePoint1, Point linePoint2) {
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + .003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 * pow(m1,2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) * x1);

        double quadraticC = ((pow(m1,2) * pow(x1,2))) - (2.0 * y1 * m1 * m1) + pow(y1,2) - pow(radius,2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {
            double xRoot1 = (-quadraticB - Math.sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2 * quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) * y1;


            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            if(xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1,yRoot1));
            }

            double xRoot2 = (-quadraticB + Math.sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) * y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2,yRoot2));
            }
        } catch(Exception e) {

        }
        return allPoints;

    }
    public static double findY(double slope, double yIntercept, double x) {
        double y = (slope * x) + yIntercept;
        return y;
    }
    public static double findSlope(double x1, double y1, double x2, double y2) {
        double slope = (y2 - y1) / (x2 - x1);
        return slope;
    }
    public static double findYIntercept(double slope, double x, double y) {
        double yIntercept = y - (slope * x);
        return yIntercept;
    }
    public static Point findSecondPoint(double slope, double distance, double x, double y, double direction) {
        double newX = x + direction * (distance / Math.sqrt((1 + Math.pow(slope, 2))));
        double yIntercept = findYIntercept(slope,x,y);
        double newY = (slope * newX) + yIntercept;
        return new Point(newX, newY);
    }
    public static double distanceBetween2Points(double x1, double y1, double x2, double y2) {
        double distance = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        return distance;
    }
    public static double distanceBetweenPointAndLine(double x, double y, double slope, double yIntercept) {
        double inverseSlope = -1/slope;
        double  yInterceptPerpendicular = findYIntercept(inverseSlope,x,y);
        double xSecondPoint = (yIntercept - yInterceptPerpendicular) / (inverseSlope - slope);
        double ySecondPoint = findY(inverseSlope,yInterceptPerpendicular,xSecondPoint);
        double distance = distanceBetween2Points(x,y,xSecondPoint, ySecondPoint);
        return distance;
    }
    public static Point secondPointAlongPerpendicularLine(double x, double y, double slope, double yIntercept) {
        double inverseSlope = -1/slope;
        double  yInterceptPerpendicular = findYIntercept(inverseSlope,x,y);
        double xSecondPoint = (yIntercept - yInterceptPerpendicular) / (inverseSlope - slope);
        double ySecondPoint = findY(inverseSlope,yInterceptPerpendicular,xSecondPoint);
        Point secondPoint = new Point(xSecondPoint, ySecondPoint);
        return secondPoint;
    }
    public static Point SubtractVectors(Point desiredVelocity, Point currentVelocity) {
        double xDiff = desiredVelocity.x * Math.cos(Math.toRadians(desiredVelocity.y)) - (currentVelocity.x * Math.cos(Math.toRadians(desiredVelocity.y)));
        double yDiff = desiredVelocity.x * Math.sin(Math.toRadians(desiredVelocity.y)) - (currentVelocity.x * Math.sin(Math.toRadians(desiredVelocity.y)));
        double magnitude = Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff,2));
        double angle = Math.toDegrees(Math.atan2(yDiff, xDiff));
        //System.out.println(yDiff);
        Point returnPoint = new Point (magnitude,angle);
        return returnPoint;
    }
    public static Point AddVectors(Point desiredVelocity, Point currentVelocity) {
        double xDiff = desiredVelocity.x * Math.cos(Math.toRadians(desiredVelocity.y)) + (currentVelocity.x * Math.cos(Math.toRadians(desiredVelocity.y)));
        double yDiff = desiredVelocity.x * Math.sin(Math.toRadians(desiredVelocity.y)) + (currentVelocity.x * Math.sin(Math.toRadians(desiredVelocity.y)));
        double magnitude = Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff,2));
        double angle = Math.toDegrees(Math.atan2(yDiff, xDiff));
        return new Point(magnitude,angle);
    }
    public static Point Limit(Point inputVector, double maxMagnitude) {
        if(inputVector.x > maxMagnitude) {
            inputVector.x = maxMagnitude;
        }
        if(inputVector.x < -maxMagnitude) {
            inputVector.x = -maxMagnitude;
        }
        return new Point(inputVector.x,inputVector.y);
    }
    public static Point Normalize(Point velocity) {
        Point returnVelocity = new Point();
        returnVelocity.x = 1;
        returnVelocity.y = velocity.y;
        return returnVelocity;
    }
    public static Point MultiplyVelocity(Point velocty, double multiplier) {
        Point returnVelocity = new Point();
        returnVelocity.x = returnVelocity.x * multiplier;
        return returnVelocity;
    }

}
