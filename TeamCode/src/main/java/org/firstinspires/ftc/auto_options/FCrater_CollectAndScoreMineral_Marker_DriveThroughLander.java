package org.firstinspires.ftc.teamcode;

public class FCrater_CollectAndScoreMineral_Marker_DriveThroughLander extends Autonomous_Parent{

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

