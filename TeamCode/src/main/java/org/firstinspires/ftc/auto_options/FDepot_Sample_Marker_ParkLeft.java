package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.teamcode.Autonomous_Parent;

public class FDepot_Sample_Marker_ParkLeft extends Autonomous_Parent {
    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        sample();
        placeTeamMarker();
        driveFromDepotToPark(true);
    }
}
