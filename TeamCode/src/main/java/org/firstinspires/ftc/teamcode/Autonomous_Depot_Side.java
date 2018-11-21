package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Depot_Side", group = "main")
public class Autonomous_Depot_Side extends Autonomous_Parent {

    @Override
    public void go() { // TODO: Either super.go() or make newly named function
        deploy();

        sampleDepotSide();

        releaseMarker();

        parkInCraterDepotSide();
    }
}
