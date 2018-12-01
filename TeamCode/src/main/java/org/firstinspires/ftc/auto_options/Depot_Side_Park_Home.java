package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;
import org.firstinspires.ftc.teamcode.TargetDirection;

public class Depot_Side_Park_Home extends Autonomous_Parent {
    @Override
    public void setup() {
        TargetDirection.setCurrentHeading(135.0);
    }

    @Override
    public void begin() {
        land();
        driveDistanceTime(60.0);
        placeTeamMarker();
        turnPID(TargetDirection.makeTargetAtFieldPosition(-90.0), 4000);
        driveDistanceTime(86.0);

    }
}
