package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Crater_Side", group = "main")
public class Autonomous_Crater_Side extends Autonomous_Parent {
    @Override
    public void getReady(){
        TargetDirection.setCurrentHeading(140.0);
    }
    //Runs autonomous
    @Override
    public void go(){
        super.go();

        deploy();

        sampleCraterSide();

        goToDepotCraterSide();

        releaseMarker();

        parkInCraterCraterSide();
    }
}
