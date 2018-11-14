package org.firstinspires.ftc.teamcode;

public class Autonomous_Depot_Side extends Autonomous_Parent {

    @Override
    public void go() {
        deploy();

        detect();

        sample();

        goToDepotCraterSide();

        parkDepotSide();

    }

}
