package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;

public class FDepot_CollectAndScoreMineral_Marker_ParkLeft extends Autonomous_Parent {

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        driveToDepot(false, true);
        scoreMineralInDepot();
        placeTeamMarker();
        driveFromDepotToPark(true);
    }
}
