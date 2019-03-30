package org.firstinspires.ftc.Interfaces;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wonderland implements DriveInterface {
    private Servo turnServo;
    private DcMotor forwardMotor;

    @Override
    public void setDrive(double forwardPower, double turnPower) {
        
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
        turnServo = hardwareMap.get(Servo.class, "ts");
        forwardMotor = hardwareMap.get(DcMotor.class, "fm");

        forwardMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turnServo.setDirection(Servo.Direction.FORWARD);

        forwardMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turnServo.setPosition(0.5);
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }
}
