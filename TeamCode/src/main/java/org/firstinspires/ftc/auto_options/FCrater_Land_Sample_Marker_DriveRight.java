package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.teamcode.Autonomous_Parent;

public class FCrater_Land_Sample_Marker_DriveRight extends Autonomous_Parent {

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        land();
        sample();
        driveFromCraterToDepot(true);
    }
}

