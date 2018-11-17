package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;

public class FCrater_CollectAndScoreMineral_Marker_DriveThroughLander extends Autonomous_Parent {

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        collectMineral();
        driveFromCraterToDepot(false);
        scoreMineralInDepot();
        placeTeamMarker();
    }
}

