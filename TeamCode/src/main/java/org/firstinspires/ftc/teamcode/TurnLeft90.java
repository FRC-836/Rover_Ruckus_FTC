package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Turn Left 90", group = "A")
public class TurnLeft90 extends Autonomous_Parent {
    @Override
    public void go() {
        driveTurnPID(-90.0);
        telemetry.addLine("Done!");
        telemetry.update();
    }
}
