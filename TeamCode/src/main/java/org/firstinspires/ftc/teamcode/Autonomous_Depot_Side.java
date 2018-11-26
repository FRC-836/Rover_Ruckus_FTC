package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Depot_Side", group = "main")
public class Autonomous_Depot_Side extends Autonomous_Parent {
    //Runs autonomous

    @Override
    public void getReady() {
        super.getReady();
        TargetDirection.setCurrentHeading(45.0);
    }

    @Override
    public void go() {
        super.go();

        deploy(135.0);

        sampleDepotSide();

        releaseMarker();

        parkInCraterDepotSide();
    }
}
