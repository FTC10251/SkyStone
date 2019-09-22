package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

public class BezierCurve {
    double leftMove;
    double rightMove;
    double middleMove;

    int side;
    int middle;

    static Point secondPoint;
    static Point thirdPoint;

    double derivative;
    static double newX = 0;
    static double newY = 0;
    static double t = 0;
    static double inverseSlope = 0;
    static double relativeAngle = 0;
    static double leftDistance = 0;
    static double rightDistance = 0;
    static public void calculateMovement (Point inputSecondPoint, Point inputThirdPoint, double tInput) {
        t = tInput;
        secondPoint = inputSecondPoint;
        thirdPoint = inputThirdPoint;
        /*double t = ((Math.sqrt((secondPoint.x * secondPoint.x) + ((thirdPoint.x - (2 * secondPoint.x)) * x))) - secondPoint.x)/(thirdPoint.x - (2 * secondPoint.x));
        double y = ((thirdPoint.y - (2 * secondPoint.y)) * (t * t)) + (2 * secondPoint.y * t);
        findDerivative(x,y);*/
    }
    static public double findDerivative(double inputT) {
        double x2 = returnXCord(inputT);
        double y2 = returnYCord(inputT);
        double x1 = returnXCord(inputT-.00001);
        double y1 = returnYCord(inputT-.00001);
        double derivative = ((y2 - y1)/(x2 - x1));
        return derivative;

    }
    static public void createSidePoints(int direction, double distance, double inputT) {
        inverseSlope = -1 * (1/findDerivative(inputT));
        relativeAngle = Math.toDegrees(Math.abs(Math.tan(inverseSlope)));
        double xChange = distance * Math.cos(Math.toRadians(relativeAngle));
        double yChange = distance * Math.sin(Math.toRadians(relativeAngle));
        newX = returnXCord(inputT) + (direction * xChange);
        newY = returnYCord(inputT) - (direction * yChange);
    }
    public double curveFunction(double x) {
        double t = ((Math.sqrt((secondPoint.x * secondPoint.x) + ((thirdPoint.x - (2 * secondPoint.x)) * x))) - secondPoint.x)/(thirdPoint.x - (2 * secondPoint.x));
        double y = ((thirdPoint.y - (2 * secondPoint.y)) * (t * t)) + (2 * secondPoint.y * t);
        return y;
    }
    static private double pointsFunction(int numOfPoints, int currentPoint) {
        double notSureWhatThisNumIs = factorial(numOfPoints)/(factorial(currentPoint) * factorial(numOfPoints - currentPoint));
        return notSureWhatThisNumIs;
    }
    static public double returnXCord(double state) {
        double xCord = findCordinateValue(state,0,0) + findCordinateValue(state,1,secondPoint.x) + findCordinateValue(state, 2,thirdPoint.x);
        return xCord;
    }
    static public double returnYCord(double state) {
        double yCord = findCordinateValue(state,0,0) + findCordinateValue(state,1,secondPoint.y) + findCordinateValue(state, 2,thirdPoint.y);
        return yCord;
    }
    static void powerRatio(double inputT) {
        double currentLeftX = getNewX(-1, inputT);
        double currentLeftY = getNewY(-1,inputT);
        double currentRightX = getNewX(1,inputT);
        double currentRightY = getNewY(1, inputT);
        double oldT = inputT - .0001;
        double previousLeftX = getNewX(-1,oldT);
        double previousLeftY = getNewY(-1,oldT);
        double previousRightX = getNewX(1,oldT);
        double previousRightY = getNewY(1,oldT);
        leftDistance = distanceBetweenPoints(currentLeftX, previousLeftX, currentLeftY, previousLeftY);
        rightDistance = distanceBetweenPoints(currentRightX, previousRightX, currentRightY, previousRightY);
    }
    static double getLeftRatio() {
        powerRatio(t);
        double leftRatio = 1;
        if(leftDistance < rightDistance) {
            leftRatio = (leftDistance / rightDistance);
        }
        return leftRatio;
    }
    static double getRightRatio() {
        powerRatio(t);
        double rightRatio = 1;
        if(rightDistance < leftDistance) {
            rightRatio = (rightDistance / leftDistance);
        }
        return rightRatio;
    }
    static public double distanceBetweenPoints(double x2, double x1, double y2, double y1) {
        double distance = Math.sqrt((Math.pow((y2 - y1),2) + Math.pow((x2 - x1),2)));
        return distance;
    }
    static private double findCordinateValue(double state, int currentPoint, double inputPoints) {
        double cordinate = 0;
        cordinate = pointsFunction(2, currentPoint) * Math.pow((1 - state), (2 - currentPoint)) * Math.pow(state,currentPoint) * inputPoints;
        return cordinate;
    }
    static int factorial(int n) {
        if (n == 0) {
            return 1;
        }
        return n*factorial(n-1);
    }
    public double findIntegral() {
        double integral = 0;
        for(double x = 0; x < 1; x = x + .00001) {
            integral = integral + (curveFunction(x) * .00001);
        }
        return integral;
    }
    static public double getNewX(int direction, double inputT) {
        createSidePoints(direction, 7.5,inputT);
        return newX;
    }
    static public double getNewY(int direction, double inputT) {
        createSidePoints(direction, 7.5,inputT);
        return newY;
    }
    public double getInverseSlope() {
        return inverseSlope;
    }
    public double getRelativeAngle() {
        return relativeAngle;
    }
    public double getLeftDistance() {
        return leftDistance;

    }
    public double getRightDistance() {
        return rightDistance;
    }
    public double getLeftDrive(){
        return leftMove;
    }
    public double getRightDrive(){
        return rightMove;
    }
    public double getMiddleDrive(){
        return middleMove;
    }
    public int getRightTicks(){ return side; }
    public int getLeftTicks() { return side; }
    public int getMiddleTicks() {return middle;
    }

}
