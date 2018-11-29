package org.firstinspires.ftc.auto_options;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;
import org.firstinspires.ftc.teamcode.TargetDirection;

@Autonomous (name = "Depot_Side")
public class Depot_Side extends Autonomous_Parent {
    @Override
    public void setup() {
        TargetDirection.setCurrentHeading(135.0);
    }

    @Override
    public void begin() {
        land();
        sampleDepot();
        placeTeamMarker();
        parkDepot();
    }
}
