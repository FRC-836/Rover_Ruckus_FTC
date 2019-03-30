package org.firstinspires.ftc.Interfaces;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Two Motor Drive")
public class SpinRobot extends OpMode {
    DriveInterface driveInterface = new TwoMotorDrive();

    @Override
    public void init() {
        driveInterface.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        driveInterface.start();
    }

    @Override
    public void loop() {
        driveInterface.setDrive(0.0,-1.0);
    }

    @Override
    public void stop() {
        driveInterface.stop();
    }


}
