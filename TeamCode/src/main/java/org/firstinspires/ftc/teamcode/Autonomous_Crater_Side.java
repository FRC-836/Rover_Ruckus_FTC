package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Crater_Side", group = "main")
public class Autonomous_Crater_Side extends Autonomous_Parent {
    //Runs autonomous

    @Override
    public void getReady() {
        super.getReady();
        TargetDirection.setCurrentHeading(135.0);
    }

    @Override
    public void go(){
        super.go();

        deploy(-135.0);

        sampleCraterSide();

        goToDepotCraterSide();

        releaseMarker();

        parkInCraterCraterSide();
    }
}
