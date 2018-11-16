package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.teamcode.Autonomous_Parent;

public class FDepot_Land_Marker_ParkLeft extends Autonomous_Parent {
    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        land();
        driveToDepot(true, false);
        placeTeamMarker();
        driveFromDepotToPark(true);
    }
}
