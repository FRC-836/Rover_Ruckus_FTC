package org.firstinspires.ftc.Interfaces;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface SubAssembly {
    void init(HardwareMap hardwareMap, Telemetry telemetry);
    void start();
    void stop();
}
