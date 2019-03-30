package org.firstinspires.ftc.Interfaces;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public class test extends OpMode {

    DriveInterface driveTrain = new SuspensionDrive();
    SubAssembly arm;

    ArrayList<SubAssembly> subAssemblies = new ArrayList<>();

    @Override
    public void init() {
        subAssemblies.add(driveTrain);
        subAssemblies.add(arm);

        for (SubAssembly subAssembly : subAssemblies)
            subAssembly.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        for (SubAssembly subAssembly : subAssemblies)
            subAssembly.start();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        for (SubAssembly subAssembly : subAssemblies)
            subAssembly.stop();
    }
}
