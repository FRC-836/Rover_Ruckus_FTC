package org.firstinspires.ftc.State_Machines_12888;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lander_And_Latch {
    private Lander_And_Latch_State_Machine stateMachine;
    private Thread thread;

    public Lander_And_Latch(HardwareMap hardwareMap, Telemetry telemetry)
    {
        stateMachine = new Lander_And_Latch_State_Machine(hardwareMap, telemetry);
        thread = new Thread(stateMachine);
    }

    public void start()
    {
        thread.start();
    }

    public void raise()
    {
        stateMachine.raise();
    }

    public void lower()
    {
        stateMachine.lower();
    }

    public void standby()
    {
        stateMachine.standby();
    }

    public void stopThread()
    {
        stateMachine.stopThread();
    }
}
