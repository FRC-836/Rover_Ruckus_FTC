package org.firstinspires.ftc.State_Machines_12888;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Team_Marker {
    private Team_Marker_State_Machine stateMachine;
    private Thread thread;

    public Team_Marker(HardwareMap hardwareMap) {
        stateMachine = new Team_Marker_State_Machine(hardwareMap);
        thread = new Thread(stateMachine);
    }

    public void start() {
        thread.start();
    }

    public void stopThread(){
        stateMachine.stopThread();
    }

    public void dropTeamMarker() {
        stateMachine.dropTeamMarker();
    }
}
