package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PurePursuitOpMode extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        RobotMovement.goToPosition(358/3,358/3,.2, Math.toRadians(90), .3);
    }
}
