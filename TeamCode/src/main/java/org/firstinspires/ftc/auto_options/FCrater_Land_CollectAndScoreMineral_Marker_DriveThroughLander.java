package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;

public class FCrater_Land_CollectAndScoreMineral_Marker_DriveThroughLander extends Autonomous_Parent {

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        land();
        collectMineral();
        driveFromCraterToDepot(false);
        scoreMineralInDepot();
        placeTeamMarker();
    }
}

