package org.firstinspires.ftc.auto_options;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;
import org.firstinspires.ftc.teamcode.TargetDirection;

@Autonomous (name = "Depot_Side_Park_Home")
public class Depot_Side_Park_Home extends Autonomous_Parent {
    @Override
    public void setup() {
        TargetDirection.setCurrentHeading(135.0);
    }

    @Override
    public void begin() {
        if (!opModeIsActive())
            return;
        land();
        if (!opModeIsActive())
            return;
        sampleDepot();
        if (!opModeIsActive())
            return;
        parkDepotHome();
    }
}
