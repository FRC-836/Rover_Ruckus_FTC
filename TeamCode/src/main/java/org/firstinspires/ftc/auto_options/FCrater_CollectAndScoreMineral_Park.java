package org.firstinspires.ftc.auto_options;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;

public class FCrater_CollectAndScoreMineral_Park extends Autonomous_Parent {
    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        collectMineral();
        driveDistance(0.0);
        //TODO:Either change to a park function or change value
    }
}
