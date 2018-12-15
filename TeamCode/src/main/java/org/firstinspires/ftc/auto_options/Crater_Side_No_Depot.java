package org.firstinspires.ftc.auto_options;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.parent_classes.Autonomous_Parent;
import org.firstinspires.ftc.teamcode.TargetDirection;

@Autonomous(name = "Crater_Side_No_Depot")
public class Crater_Side_No_Depot extends Autonomous_Parent {
    @Override
    public void setup() {
        TargetDirection.setCurrentHeading(-135.0);
    }

    @Override
    public void begin() {
        if (!opModeIsActive())
            return;
        land();
        if (!opModeIsActive())
            return;
        sampleParkCrater();
    }
}
