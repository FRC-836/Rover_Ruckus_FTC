package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Turn Right 90", group = "A")
public class TurnRight90 extends Autonomous_Parent {
    @Override
    public void go() {
        driveTurnPID(90.0);
        telemetry.addLine("Done!");
        telemetry.update();
    }
}
