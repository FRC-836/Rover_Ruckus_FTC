package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Strafe Left 12", group = "A")
public class StrafeLeft12 extends Autonomous_Parent {
    @Override
    public void go() {
        driveStrafePID(-12.0);
        telemetry.addLine("Done!");
        telemetry.update();
    }
}
