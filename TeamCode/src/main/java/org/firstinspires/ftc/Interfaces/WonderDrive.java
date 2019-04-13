package org.firstinspires.ftc.Interfaces;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class WonderDrive implements DriveInterface {
    private ArrayList<DcMotor> dcMotorList = new ArrayList<DcMotor>();
    private CRServo turnServo;
    private int n;

    public WonderDrive() {
        n = 1;
    }

    public WonderDrive(int n) {
        this.n = n;
    }

    @Override
    public void setDrive(double forwardPower, double turnPower) {
        for (DcMotor motor : dcMotorList) {
            motor.setPower(forwardPower);
        }
        turnServo.setPower(turnPower / 5.0);
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
        for (int i = 1; i <= n; i++) {
            // TODO: Figure out different motor names
            DcMotor motor = hardwareMap.get(DcMotor.class, "drive");
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            dcMotorList.add(motor);
        }

        turnServo = hardwareMap.get(CRServo.class, "steer");
        turnServo.setDirection(CRServo.Direction.REVERSE);
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        setDrive(0.0, 0.0);
    }
}
