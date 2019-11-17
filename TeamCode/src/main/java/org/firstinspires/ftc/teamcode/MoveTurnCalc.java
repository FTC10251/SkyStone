package org.firstinspires.ftc.teamcode;

    public class MoveTurnCalc {
        // will turn clockwise
        // put in negative for angle turn if your want counter clockwise
        double middlePower = 0;
        double leftYPower = 0;
        double rightYPower = 0;
        HDriveFCCalc calculator;

        public MoveTurnCalc() {
            calculator = new HDriveFCCalc();
        }

        public void calculateMove(double x_inches, double y_inches, double angleTurn, double currentAngle){
            double xJoy;
            double yJoy;
            double turnJoy;
            double turnInches = angleTurn*(Math.PI/180)*9;
            if (Math.abs(x_inches) >= Math.abs(y_inches) && Math.abs(x_inches) >= Math.abs(turnInches)) {
                yJoy = y_inches/Math.abs(x_inches);
                turnJoy = turnInches/Math.abs(x_inches);
                xJoy = x_inches/Math.abs(x_inches);
            } else if (Math.abs(y_inches)>=Math.abs(x_inches) && Math.abs(y_inches) >= Math.abs(turnInches)){
               xJoy = x_inches/Math.abs(y_inches);
               turnJoy = turnInches/Math.abs(y_inches);
               yJoy = y_inches/Math.abs(y_inches);
            } else{
                xJoy = x_inches/Math.abs(turnInches);
                yJoy = y_inches/Math.abs(turnInches);
                turnJoy = turnInches/Math.abs(turnInches);

            }

            calculator.calculateMovement2(xJoy, yJoy, turnJoy , currentAngle);
            middlePower = calculator.getMiddleDrive() * 2;
            leftYPower = calculator.getLeftDrive();
            rightYPower = calculator.getRightDrive();
            if (middlePower > 1){
                leftYPower = leftYPower/middlePower;
                rightYPower = rightYPower/middlePower;
            }
        }

        public double getLeftYPower() {
            return leftYPower;
        }

        public double getMiddlePower() {
            return middlePower;
        }

        public double getRightYPower() {
            return rightYPower;
        }
    }
