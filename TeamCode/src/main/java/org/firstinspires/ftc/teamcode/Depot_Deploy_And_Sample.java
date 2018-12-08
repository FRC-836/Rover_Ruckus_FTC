package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Depot_Side", group = "main")
public class Depot_Deploy_And_Sample extends Autonomous_Parent {
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

        sampleDepotSide();

        releaseMarker();
    }
}
