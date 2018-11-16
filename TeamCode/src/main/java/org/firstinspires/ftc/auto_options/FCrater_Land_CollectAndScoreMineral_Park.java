package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.teamcode.Autonomous_Parent;

public class FCrater_Land_CollectAndScoreMineral_Park extends Autonomous_Parent {
    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        land();
        collectMineral();
        driveDistance(0.0);
        //TODO:Either change to a park function or change value
    }
}
