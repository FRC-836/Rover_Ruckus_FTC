package org.firstinspires.ftc.teamcode;

public class FDepot_Land_CollectAndScoreMineral_Marker extends Autonomous_Parent{

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        land();
        driveToDepot(false, true);
        scoreMineralInDepot();
        placeTeamMarker();
    }
}
