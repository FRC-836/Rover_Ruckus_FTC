package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Functions", group = "A")
public class Functions_Test_3 extends Autonomous_Parent {
    @Override
    public void go() {
        driveDistancePID(12);
        waitSeconds(0.5);
        driveDistancePID(-12);
        waitSeconds(0.5);
        driveStrafePID(12);
        waitSeconds(0.5);
        driveStrafePID(-12);
        waitSeconds(0.5);
        driveTurnPID(90);
        waitSeconds(0.5);
        driveTurnPID(-90);
        waitSeconds(0.5);

        driveDistance(6);
        driveStrafe(6);
        driveTurn(180);
        driveDistance(6);


    }
}
