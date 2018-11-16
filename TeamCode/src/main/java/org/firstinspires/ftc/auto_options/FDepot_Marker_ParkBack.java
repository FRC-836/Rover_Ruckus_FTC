package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.teamcode.Autonomous_Parent;

public class FDepot_Marker_ParkBack extends Autonomous_Parent {
    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        driveToDepot(false, false);
        placeTeamMarker();
        driveFromDepotToPark(false);
    }
}