package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Robot Functions Test (dance)", group = "A")
public class Robot_Functions_Test_4 extends Autonomous_Parent {
    @Override
    public void go() {
        driveDistance(0.5);
        
        driveDistance(-0.5);
        driveStrafe(5);
        driveTurn(90);
        driveDistance(0.25);
        driveDistance(-0.25);
        driveDistance(0.25);
        driveDistance(-0.25);
        driveDistancePID(12);
        driveStrafePID(-12);
        driveTurnPID(135);

        waitSeconds(2);

        for (int i = 0; i < 300; i++) {setDrive(0.1,0.5,1.0); sleep(5);}
    }
}
