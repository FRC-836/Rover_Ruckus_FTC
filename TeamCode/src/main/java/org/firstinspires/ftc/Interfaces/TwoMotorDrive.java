package org.firstinspires.ftc.Interfaces;

import android.drm.DrmStore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TwoMotorDrive implements DriveInterface {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void setDrive(double forwardPower, double turnPower) {
        leftMotor.setPower(forwardPower + turnPower);
        rightMotor.setPower(forwardPower - turnPower);
    }

    @Override
    public void driveDistance(double inches) {

    }

    @Override
    public void turnDegrees(double degrees) {

    }

    @Override
    public boolean isDoneMoving() {
        return false;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotor.class, "lm");
        rightMotor = hardwareMap.get(DcMotor.class, "rm");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
     leftMotor.setPower(0.0);
     rightMotor.setPower(0.0);
    }
}
