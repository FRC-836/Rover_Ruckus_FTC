package org.firstinspires.ftc.teamcode;

public class FCrater_Land_Sample_Marker_DriveThroughLander extends Autonomous_Parent{

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        land();
        sample();
        driveFromCraterToDepot(false);
    }
}
