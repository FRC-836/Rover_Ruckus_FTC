package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Forward 12", group = "A")
public class Forward12 extends Autonomous_Parent {
    @Override
    public void go() {
        driveDistancePID(12.0);
        telemetry.addLine("Done!");
        telemetry.update();
    }
}
