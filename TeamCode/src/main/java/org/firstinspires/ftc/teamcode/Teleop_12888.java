package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.State_Machines_12888.Team_Marker_State_Machine;

@TeleOp(name = "State Test 12888")
public class Teleop_12888 extends LinearOpMode {
    private Team_Marker_State_Machine teamMarker;

    @Override
    public void runOpMode() throws InterruptedException {
        teamMarker = new Team_Marker_State_Machine(hardwareMap);
        Thread teamMarkerThread = new Thread(teamMarker);

        waitForStart();

        teamMarkerThread.start();

        while (opModeIsActive())
        {
            if (gamepad1.a)
            {
                teamMarker.dropTeamMarker();
            }
            sleep(0);
        }

        teamMarker.stop();
    }
}
