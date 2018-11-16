package org.firstinspires.ftc.teamcode;


public class FDepot_Land_Sample_Marker_ParkLeft extends Autonomous_Parent{
    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        land();
        sample();
        placeTeamMarker();
        driveFromDepotToPark(true);
    }
}
