package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Crater Deploy and Sample")
public class Crater_Deploy_And_Sample extends Autonomous_Parent{
    @Override
    public void getReady(){
        super.getReady();
        TargetDirection.setCurrentHeading(135.0);
    }
    //Runs autonomous
    @Override
    public void go() {
        super.go();

        deploy();

        sampleCraterSide();
    }
}
