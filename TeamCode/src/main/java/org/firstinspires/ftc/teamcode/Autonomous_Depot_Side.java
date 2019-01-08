package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Depot_Side", group = "main")
public class Autonomous_Depot_Side extends Autonomous_Parent {
    @Override
    public void getReady(){
        super.getReady();
        TargetDirection.setCurrentHeading(45.0);
    }
    //Runs autonomous
    @Override
    public void go() {
        super.go();

        deploy();

        if(!opModeIsActive())
            return;

        sampleDepotSide();

        if(!opModeIsActive())
            return;

        releaseMarker();

        if(!opModeIsActive())
            return;

        parkInCraterDepotSide();
    }
}
