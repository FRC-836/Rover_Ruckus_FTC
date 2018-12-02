package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Backward 12", group = "A")
public class Backward12 extends Autonomous_Parent {
    @Override
    public void go() {
        driveDistancePID(-12.0);
        telemetry.addLine("Done!");
        telemetry.update();
    }
}
