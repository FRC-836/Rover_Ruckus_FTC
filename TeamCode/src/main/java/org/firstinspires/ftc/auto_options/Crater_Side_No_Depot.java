package org.firstinspires.ftc.auto_options;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;

@Autonomous(name = "Crater_Side_No_Depot")
public class Crater_Side_No_Depot extends Autonomous_Parent {
    @Override
    public void setup() {

    }

    @Override
    public void begin() {
        land();
        sampleParkCrater();
    }
}
