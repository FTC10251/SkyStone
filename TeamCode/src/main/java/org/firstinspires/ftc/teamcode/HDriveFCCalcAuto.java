package org.firstinspires.ftc.teamcode;

public class HDriveFCCalcAuto {
        double leftMove;
        double rightMove;
        double middleMove;

        int side;
        int middle;
        public void calculateMovement (double moveAngle, double angle, double inches) {
            double theta = moveAngle/360 * 2 * Math.PI;

            double robotAngle = -angle/360 * 2 * Math.PI;
            double tempVal = Math.sin(theta) * Math.cos(robotAngle) - Math.cos(theta) * Math.sin(robotAngle);
            double driveMiddle = Math.cos(theta) * Math.cos(-robotAngle) - Math.sin(theta) * Math.sin(-robotAngle);
            double driveLeft = tempVal;
            double driveRight = tempVal;

            int encoderTicks = (int) ((1150 * inches)/ (2 * Math.PI * 2));
            side = encoderTicks * (int) tempVal;
            middle = encoderTicks * (int) driveMiddle;


            if (driveLeft > 1) {
                driveRight = driveRight * (1 / driveLeft);
                driveMiddle = driveMiddle * (1 / driveLeft);
                driveLeft = 1;
            } else if (driveRight > 1) {
                driveLeft = driveLeft * (1 / driveRight);
                driveMiddle = driveMiddle * (1 / driveRight);
                driveRight = 1;
            }

            //scaling
            if (Math.abs(driveLeft) > Math.abs(driveMiddle) && Math.abs(driveLeft) > Math.abs(driveRight)){
                driveRight = driveRight/ (1/Math.abs(driveLeft));
                driveMiddle = driveMiddle / (1/Math.abs(driveLeft));
                driveLeft = driveLeft/ (1/Math.abs(driveLeft));

            } else if (Math.abs(driveRight)> Math.abs(driveMiddle) && Math.abs(driveRight) > Math.abs(driveLeft)){
                driveLeft = driveLeft / (1/Math.abs(driveRight));
                driveMiddle = driveMiddle / (1/Math.abs(driveRight));
                driveRight = driveRight / (1/Math.abs(driveRight));

            } else if (Math.abs(driveMiddle)> Math.abs(driveLeft) && Math.abs(driveMiddle) > Math.abs(driveRight)){
                driveRight = driveRight / (1/Math.abs(driveRight));
                driveLeft = driveLeft / (1/Math.abs(driveRight));
                driveMiddle = driveMiddle / (1/Math.abs(driveMiddle));
            }

            leftMove = driveLeft;
            rightMove = driveRight;
            middleMove = driveMiddle;
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
