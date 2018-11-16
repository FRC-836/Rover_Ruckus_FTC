package org.firstinspires.ftc.teamcode;

public class FDepot_CollectAndScoreMineral_Marker_ParkLeft extends Autonomous_Parent{

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
