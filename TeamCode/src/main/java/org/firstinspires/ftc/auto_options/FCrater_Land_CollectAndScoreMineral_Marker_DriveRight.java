package org.firstinspires.ftc.teamcode;

public class FCrater_Land_CollectAndScoreMineral_Marker_DriveRight extends Autonomous_Parent{

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        land();
        collectMineral();
        driveFromCraterToDepot(true);
        scoreMineralInDepot();
        placeTeamMarker();
    }
}

