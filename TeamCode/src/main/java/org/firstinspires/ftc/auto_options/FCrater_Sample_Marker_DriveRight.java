package org.firstinspires.ftc.teamcode;

public class FCrater_Sample_Marker_DriveRight extends Autonomous_Parent{

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        sample();
        driveFromCraterToDepot(true);
    }
}
