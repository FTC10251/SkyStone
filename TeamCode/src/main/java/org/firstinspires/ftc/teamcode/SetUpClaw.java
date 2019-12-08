package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.SkyStoneAutonomousUpdated;

@Autonomous(name = "Claw Setup")
public class SetUpClaw extends LinearOpMode {
    Servo clawServo;
    Servo holdServo;
    Servo rotationServo;
    Servo leftIntakeServo;
    @Override
    public void runOpMode() throws InterruptedException {
        clawServo = hardwareMap.get(Servo .class, "Claw Servo");
        holdServo = hardwareMap.get(Servo.class, "Hold Servo");
        rotationServo = hardwareMap.get(Servo.class, "Rotation Servo");
        leftIntakeServo = hardwareMap.get(Servo.class, "Intake Servo Left");
        waitForStart();
        while(opModeIsActive()) {
            clawServo.setPosition(SkyStoneAutonomousUpdated.servoOpenPos);
            holdServo.setPosition(SkyStoneAutonomousUpdated.holdServoPos);
            rotationServo.setPosition(.6);
            leftIntakeServo.setPosition(1);
        }
    }
}
