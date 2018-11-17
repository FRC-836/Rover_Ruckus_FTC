package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Crater_Side", group = "main")
public class Autonomous_Crater_Side extends Autonomous_Parent {

    @Override
    public void go() {
        deploy();

        sample();

        goToDepotCraterSide();

        releaseMarker();

        parkCraterSide();

    }
}
