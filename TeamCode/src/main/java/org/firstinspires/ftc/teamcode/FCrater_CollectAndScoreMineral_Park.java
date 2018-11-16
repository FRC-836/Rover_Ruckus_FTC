package org.firstinspires.ftc.teamcode;

public class FCrater_CollectAndScoreMineral_Park extends Autonomous_Parent{
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
