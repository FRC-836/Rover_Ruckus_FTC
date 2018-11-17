package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;

public class FCrater_Sample_Marker_DriveRight extends Autonomous_Parent {

    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        sample();
        driveFromCraterToDepot(true);
    }
}
